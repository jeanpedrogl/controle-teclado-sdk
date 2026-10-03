"""Lógica de controle: teclas e mouse -> comandos de voo normalizados.

Python puro: sem ROS2 e sem pygame, para poder ser testado sem janela nem drone.
`piloto_teclado_mouse.py` traduz eventos do pygame para as chamadas daqui e publica o resultado.

Convenções de eixo (REP 103, padrão ROS): x = frente, y = esquerda, z = cima, yaw positivo =
anti-horário. O celular converte para o que o SDK da DJI espera. Todos os valores saem em [-1, 1];
o app multiplica pelos limites reais (m/s, graus/s).
"""

from dataclasses import dataclass
import math

# --- Sensação de pilotagem (ajuste aqui) -------------------------------------------------------

# Tempo para ir de 0 a 1 ao apertar uma tecla, e de 1 a 0 ao soltar. WASD é liga/desliga; sem a
# rampa o drone receberia degraus de velocidade.
TEMPO_ACELERAR_S = 0.30
TEMPO_FREAR_S = 0.15

# Mouse (arrasto com o botão esquerdo): o deslocamento desde onde você clicou vira velocidade,
# como um joystick. Dentro da zona morta nada acontece (tremor da mão); em ALCANCE_PX vale 1.
ZONA_MORTA_PX = 10
ALCANCE_PX = 150
EXPOENTE = 1.6  # >1: movimentos pequenos dão comandos bem pequenos, os grandes chegam ao máximo
SUAVIZACAO_S = 0.08  # constante de tempo do filtro passa-baixa do mouse

# O arrasto "puxa a imagem": arrastar para baixo faz a imagem descer, ou seja, a câmera sobe;
# arrastar para a direita faz a imagem ir para a direita, ou seja, o drone gira para a esquerda.
# True inverte só o gimbal (arrastar para baixo = câmera desce, como olhar para baixo).
INVERTER_GIMBAL = False

# Setas cima/baixo inclinam a câmera (gimbal) a esta fração da velocidade máxima. 1 = o máximo que o
# celular aceita (MAX_GIMBAL_DPS no app); mais baixo dá um ajuste mais fino. Somam com o mouse.
VELOCIDADE_GIMBAL_TECLAS = 0.85

# Setas esquerda/direita giram o drone em torno do próprio eixo (guinada) a esta fração da
# velocidade máxima de giro. Somam com o arrasto horizontal do mouse; só valem armado.
VELOCIDADE_GUINADA_TECLAS = 0.85

# Ações perigosas só disparam depois da tecla ficar apertada por este tempo.
SEGURAR_ACAO_S = 0.8


@dataclass
class Comando:
    """Saída de `Piloto.passo`. Todos em [-1, 1]."""

    x: float = 0.0  # frente (+) / trás (-)
    y: float = 0.0  # esquerda (+) / direita (-)
    z: float = 0.0  # subir (+) / descer (-)
    yaw: float = 0.0  # anti-horário (+) / horário (-)
    gimbal: float = 0.0  # câmera sobe (+) / desce (-)


def limitar(v: float, minimo: float = -1.0, maximo: float = 1.0) -> float:
    return max(minimo, min(maximo, v))


def normalizar_horizontal(frente: float, esquerda: float) -> tuple[float, float]:
    """Impede que a diagonal (W+D) seja mais rápida que um eixo sozinho."""
    modulo = math.hypot(frente, esquerda)
    if modulo <= 1.0:
        return frente, esquerda
    return frente / modulo, esquerda / modulo


def eixo_mouse(delta_px: float) -> float:
    """Deslocamento do mouse em pixels -> [-1, 1], com zona morta e curva exponencial."""
    modulo = abs(delta_px)
    if modulo <= ZONA_MORTA_PX:
        return 0.0
    v = min((modulo - ZONA_MORTA_PX) / (ALCANCE_PX - ZONA_MORTA_PX), 1.0) ** EXPOENTE
    return math.copysign(v, delta_px)


class Rampa:
    """Leva um valor até o alvo com limite de variação: sobe devagar, freia mais rápido."""

    def __init__(self) -> None:
        self.valor = 0.0

    def atualizar(self, alvo: float, dt: float) -> float:
        freando = abs(alvo) < abs(self.valor) or alvo * self.valor < 0
        tempo = TEMPO_FREAR_S if freando else TEMPO_ACELERAR_S
        passo = dt / tempo
        if alvo > self.valor:
            self.valor = min(alvo, self.valor + passo)
        else:
            self.valor = max(alvo, self.valor - passo)
        return self.valor

    def zerar(self) -> None:
        self.valor = 0.0


class Filtro:
    """Passa-baixa de 1ª ordem; independente da taxa de atualização."""

    def __init__(self, tau_s: float) -> None:
        self.tau_s = tau_s
        self.valor = 0.0

    def atualizar(self, alvo: float, dt: float) -> float:
        alfa = 1.0 - math.exp(-dt / self.tau_s) if dt > 0 else 0.0
        self.valor += (alvo - self.valor) * alfa
        return self.valor

    def zerar(self) -> None:
        self.valor = 0.0


class StickMouse:
    """Arrastar o mouse como um joystick: horizontal = guinada, vertical = gimbal.

    O ponto do clique é o centro; soltar o botão zera. É como puxar a imagem: arrastar para a
    direita gira o drone para a esquerda (anti-horário, yaw positivo em REP 103) e arrastar para
    baixo inclina a câmera para cima.
    """

    def __init__(self) -> None:
        self.ancora: tuple[float, float] | None = None
        self.atual: tuple[float, float] = (0.0, 0.0)

    @property
    def ativo(self) -> bool:
        return self.ancora is not None

    def pressionar(self, pos: tuple[float, float]) -> None:
        self.ancora = pos
        self.atual = pos

    def mover(self, pos: tuple[float, float]) -> None:
        if self.ancora is not None:
            self.atual = pos

    def soltar(self) -> None:
        self.ancora = None

    def alvo(self) -> tuple[float, float]:
        """(yaw, gimbal) instantâneos, sem filtro."""
        if self.ancora is None:
            return 0.0, 0.0
        dx = self.atual[0] - self.ancora[0]
        dy = self.atual[1] - self.ancora[1]  # a tela cresce para baixo
        yaw = eixo_mouse(dx)
        gimbal = eixo_mouse(dy)
        return yaw, (-gimbal if INVERTER_GIMBAL else gimbal)


class AcaoSegurada:
    """Dispara uma vez por aperto, depois da tecla ficar apertada por `duracao_s`."""

    def __init__(self, duracao_s: float = SEGURAR_ACAO_S) -> None:
        self.duracao_s = duracao_s
        self._inicio: dict[str, float] = {}
        self._disparadas: set[str] = set()

    def pressionar(self, nome: str, agora: float) -> None:
        self._inicio.setdefault(nome, agora)

    def soltar(self, nome: str) -> None:
        self._inicio.pop(nome, None)
        self._disparadas.discard(nome)

    def soltar_tudo(self) -> None:
        self._inicio.clear()
        self._disparadas.clear()

    def progresso(self, agora: float) -> dict[str, float]:
        """Fração [0, 1] de cada ação em andamento (para mostrar na tela)."""
        return {
            n: min((agora - t) / self.duracao_s, 1.0)
            for n, t in self._inicio.items()
            if n not in self._disparadas
        }

    def atualizar(self, agora: float) -> list[str]:
        prontas = [
            n for n, t in self._inicio.items()
            if n not in self._disparadas and agora - t >= self.duracao_s
        ]
        self._disparadas.update(prontas)
        return prontas


class Piloto:
    """Junta teclas, mouse e o estado armado/desarmado em um `Comando` por passo de tempo.

    Teclas (nomes lógicos): w, a, s, d, espaco, shift (voo), esquerda, direita (guinada) e cima, baixo (gimbal).

    Desarmado: nada de movimento nem de guinada é enviado (a rampa é zerada). O gimbal funciona
    mesmo desarmado, porque a câmera não faz parte dos controles de voo.
    """

    def __init__(self) -> None:
        self.teclas: set[str] = set()
        self.mouse = StickMouse()
        self.armado = False
        self._x = Rampa()
        self._y = Rampa()
        self._z = Rampa()
        self._yaw = Filtro(SUAVIZACAO_S)
        self._gimbal = Filtro(SUAVIZACAO_S)

    def tecla_pressionada(self, nome: str) -> None:
        self.teclas.add(nome)

    def tecla_solta(self, nome: str) -> None:
        self.teclas.discard(nome)

    def soltar_tudo(self) -> None:
        """Foco perdido ou janela fechando: nenhuma tecla/botão pode ficar 'preso'."""
        self.teclas.clear()
        self.mouse.soltar()

    def armar(self) -> None:
        self.armado = True

    def desarmar(self) -> None:
        self.armado = False
        for r in (self._x, self._y, self._z):
            r.zerar()
        self._yaw.zerar()

    def passo(self, dt: float) -> Comando:
        yaw_mouse, gimbal_mouse = self.mouse.alvo()
        gimbal_teclas = (float("cima" in self.teclas) - float("baixo" in self.teclas)) * VELOCIDADE_GIMBAL_TECLAS
        gimbal = self._gimbal.atualizar(limitar(gimbal_mouse + gimbal_teclas), dt)
        yaw_teclas = (float("esquerda" in self.teclas) - float("direita" in self.teclas)) * VELOCIDADE_GUINADA_TECLAS
        yaw_alvo = limitar(yaw_mouse + yaw_teclas)
        if not self.armado:
            return Comando(gimbal=_zero_se_pequeno(gimbal))

        frente = float("w" in self.teclas) - float("s" in self.teclas)
        esquerda = float("a" in self.teclas) - float("d" in self.teclas)
        sobe = float("espaco" in self.teclas) - float("shift" in self.teclas)
        frente, esquerda = normalizar_horizontal(frente, esquerda)
        return Comando(
            x=_zero_se_pequeno(self._x.atualizar(frente, dt)),
            y=_zero_se_pequeno(self._y.atualizar(esquerda, dt)),
            z=_zero_se_pequeno(self._z.atualizar(sobe, dt)),
            yaw=_zero_se_pequeno(self._yaw.atualizar(yaw_alvo, dt)),
            gimbal=_zero_se_pequeno(gimbal),
        )


def _zero_se_pequeno(v: float, limite: float = 0.005) -> float:
    """O filtro nunca chega exatamente a 0; sem isso o comando ficaria em ±0.0001 para sempre."""
    return 0.0 if abs(v) < limite else v
