import math

from ponte_teclado_drone import controle
from ponte_teclado_drone.controle import (
    ALCANCE_PX, ZONA_MORTA_PX, AcaoSegurada, Piloto, Rampa, StickMouse, eixo_mouse,
    normalizar_horizontal,
)

DT = 0.05


def rodar(piloto: Piloto, segundos: float):
    comando = None
    for _ in range(round(segundos / DT)):
        comando = piloto.passo(DT)
    return comando


def test_diagonal_nao_passa_de_um():
    x, y = normalizar_horizontal(1.0, 1.0)
    assert math.isclose(math.hypot(x, y), 1.0)
    assert normalizar_horizontal(1.0, 0.0) == (1.0, 0.0)


def test_rampa_sobe_em_tempo_acelerar_e_freia_mais_rapido():
    r = Rampa()
    for _ in range(round(controle.TEMPO_ACELERAR_S / DT)):
        r.atualizar(1.0, DT)
    assert math.isclose(r.valor, 1.0)
    assert r.atualizar(0.0, DT) < 1.0
    passos = 0
    while r.valor > 0:
        r.atualizar(0.0, DT)
        passos += 1
    assert passos * DT <= controle.TEMPO_FREAR_S + DT


def test_zona_morta_e_alcance_do_mouse():
    assert eixo_mouse(ZONA_MORTA_PX) == 0.0
    assert eixo_mouse(-ZONA_MORTA_PX + 1) == 0.0
    assert eixo_mouse(ALCANCE_PX) == 1.0
    assert eixo_mouse(10 * ALCANCE_PX) == 1.0
    assert eixo_mouse(-ALCANCE_PX) == -1.0
    # monotônico e suave perto da zona morta (movimento pequeno = comando bem pequeno)
    assert 0.0 < eixo_mouse(ZONA_MORTA_PX + 5) < 0.1
    assert eixo_mouse(60) < eixo_mouse(90) < eixo_mouse(120)


def test_arrastar_puxa_a_imagem_direita_gira_esquerda_e_baixo_sobe_a_camera():
    s = StickMouse()
    s.pressionar((100, 100))
    s.mover((250, 100))
    yaw, gimbal = s.alvo()
    assert yaw > 0 and gimbal == 0  # imagem vai à direita: drone gira à esquerda (anti-horário = yaw +)
    s.mover((100, 250))
    yaw, gimbal = s.alvo()
    assert yaw == 0 and gimbal > 0  # imagem desce: câmera sobe
    s.soltar()
    assert s.alvo() == (0.0, 0.0)


def test_mover_sem_pressionar_nao_faz_nada():
    s = StickMouse()
    s.mover((500, 500))
    assert s.alvo() == (0.0, 0.0)


def test_desarmado_nao_manda_voo_mas_manda_gimbal():
    p = Piloto()
    p.tecla_pressionada("w")
    p.mouse.pressionar((0, 0))
    p.mouse.mover((200, 200))
    c = rodar(p, 1.0)
    assert (c.x, c.y, c.z, c.yaw) == (0, 0, 0, 0)
    assert c.gimbal > 0


def test_wasd_espaco_shift_armado():
    p = Piloto()
    p.armar()
    for t in ("w", "a", "espaco"):
        p.tecla_pressionada(t)
    c = rodar(p, 1.0)
    assert c.x > 0 and c.y > 0 and c.z > 0
    assert math.isclose(math.hypot(c.x, c.y), 1.0, rel_tol=1e-6)
    for t in ("w", "a", "espaco"):
        p.tecla_solta(t)
    for t in ("s", "d", "shift"):
        p.tecla_pressionada(t)
    c = rodar(p, 1.0)
    assert c.x < 0 and c.y < 0 and c.z < 0


def test_comando_sobe_em_rampa_e_nao_em_degrau():
    p = Piloto()
    p.armar()
    p.tecla_pressionada("w")
    primeiro = p.passo(DT)
    assert 0 < primeiro.x < 0.5


def test_soltar_tudo_para_o_drone():
    p = Piloto()
    p.armar()
    p.tecla_pressionada("w")
    p.mouse.pressionar((0, 0))
    p.mouse.mover((200, 0))
    rodar(p, 1.0)
    p.soltar_tudo()
    c = rodar(p, 1.0)
    assert (c.x, c.y, c.z, c.yaw, c.gimbal) == (0, 0, 0, 0, 0)


def test_desarmar_zera_na_hora():
    p = Piloto()
    p.armar()
    p.tecla_pressionada("w")
    rodar(p, 1.0)
    p.desarmar()
    c = p.passo(DT)
    assert c.x == 0 and c.yaw == 0
    p.armar()
    assert p.passo(DT).x < 0.5  # recomeça pela rampa, sem herdar a velocidade antiga


def test_acao_segurada_dispara_uma_vez_so():
    a = AcaoSegurada(duracao_s=0.8)
    a.pressionar("l", 0.0)
    assert a.atualizar(0.5) == []
    assert 0.5 < a.progresso(0.5)["l"] < 0.7
    assert a.atualizar(0.9) == ["l"]
    assert a.atualizar(2.0) == []  # não repete enquanto a tecla continua apertada
    a.soltar("l")
    a.pressionar("l", 3.0)
    assert a.atualizar(3.9) == ["l"]


def test_acao_soltar_antes_cancela():
    a = AcaoSegurada(duracao_s=0.8)
    a.pressionar("h", 0.0)
    a.soltar("h")
    assert a.atualizar(5.0) == []


def test_setas_inclinam_a_camera_mesmo_desarmado():
    p = Piloto()
    p.tecla_pressionada("cima")
    c = rodar(p, 1.0)
    assert c.gimbal > 0 and (c.x, c.y, c.z, c.yaw) == (0, 0, 0, 0)
    p.tecla_solta("cima")
    p.tecla_pressionada("baixo")
    assert rodar(p, 1.0).gimbal < 0
    p.tecla_solta("baixo")
    assert rodar(p, 1.0).gimbal == 0


def test_cima_e_baixo_juntas_se_anulam_e_gimbal_fica_limitado():
    p = Piloto()
    p.tecla_pressionada("cima")
    p.tecla_pressionada("baixo")
    assert rodar(p, 1.0).gimbal == 0
    p.tecla_solta("baixo")
    p.mouse.pressionar((0, 0))
    p.mouse.mover((0, 300))
    assert rodar(p, 2.0).gimbal <= 1.0


def test_setas_esquerda_direita_giram_so_armado():
    p = Piloto()
    p.tecla_pressionada("esquerda")
    assert rodar(p, 1.0).yaw == 0  # desarmado: nada de guinada
    p.armar()
    assert rodar(p, 1.0).yaw > 0  # esquerda = anti-horário = yaw +
    p.tecla_solta("esquerda")
    p.tecla_pressionada("direita")
    assert rodar(p, 1.0).yaw < 0
    p.tecla_pressionada("esquerda")
    assert rodar(p, 1.0).yaw == 0  # as duas juntas se anulam
