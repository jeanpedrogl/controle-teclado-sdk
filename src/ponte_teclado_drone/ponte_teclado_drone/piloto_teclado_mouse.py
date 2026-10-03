"""Pilota o drone pelo teclado e mouse e mostra o vídeo do drone na mesma janela.

Nó ROS2 + janela pygame. Toda a lógica de controle está em `controle.py` (testada sem ROS2);
aqui só se traduz evento -> chamada e comando -> mensagem. O celular (app_proprio) assina os
tópicos via rosbridge; veja o PROTOCOLO.md daquele projeto.

Teclado (janela em foco):
    W A S D       frente / esquerda / trás / direita (plano paralelo ao solo)
    Espaço/Shift  subir / descer
    Mouse         arrastar com o botão esquerdo, como puxar a imagem: horizontal = girar o drone,
                  vertical = gimbal (arrastar para a direita gira para a esquerda; para baixo, a
                  câmera sobe)
    Enter         arma/desarma o envio de comandos de voo
    Esc           desarma na hora (o controle físico da DJI volta a mandar)
    Setas ← / →   gira o drone para a esquerda / direita (guinada)
    Setas ↑ / ↓   câmera (gimbal) sobe / desce
    T / L / H / G segurar ~1 s: decolar / pousar / voltar para casa / cancelar pouso ou retorno
    1 2 3 4       qualidade do vídeo: desempenho / equilibrado / qualidade / máximo
    Ctrl+Q        sai
"""

import collections
import io
import json
import time

import pygame
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

from .controle import AcaoSegurada, Comando, Piloto

LARGURA, ALTURA = 960, 540
PERIODO_ENVIO_S = 0.05  # 20 Hz; o app zera tudo se ficar 500 ms sem receber
ZEROS_AO_PARAR = 3  # zeros seguidos no gimbal ao soltar o mouse (uma mensagem pode se perder)
SEM_TELEMETRIA_S = 3.0
# Depois de armar, o celular leva um instante para ligar o stick mode e a telemetria chega a 2 Hz;
# só depois disso um "stick_mode: false" vindo dele é tratado como "o celular saiu do modo".
GRACA_ARME_S = 3.0

TECLAS_MOVIMENTO = {
    pygame.K_w: "w",
    pygame.K_a: "a",
    pygame.K_s: "s",
    pygame.K_d: "d",
    pygame.K_SPACE: "espaco",
    pygame.K_LSHIFT: "shift",
    pygame.K_RSHIFT: "shift",
    pygame.K_LEFT: "esquerda",  # guinada: gira para a esquerda (anti-horário)
    pygame.K_RIGHT: "direita",  # guinada: gira para a direita (horário)
    pygame.K_UP: "cima",  # gimbal: câmera sobe
    pygame.K_DOWN: "baixo",  # gimbal: câmera desce
}

# Qualidade do vídeo que o celular reenvia (tecla -> rótulo, argumentos de `video.set_quality`).
# Mais fps/pixels/qualidade = imagem melhor, mas mais tráfego no Wi-Fi e mais CPU no celular; se
# a imagem engasgar, desça um preset. Valores iniciais: ajuste conforme o desempenho real.
PRESETS_VIDEO = {
    pygame.K_1: ("Desempenho", {"fps": 12, "max_width": 480, "jpeg_quality": 35}),
    pygame.K_2: ("Equilibrado", {"fps": 8, "max_width": 640, "jpeg_quality": 50}),
    pygame.K_3: ("Qualidade", {"fps": 8, "max_width": 960, "jpeg_quality": 75}),
    pygame.K_4: ("Máximo", {"fps": 20, "max_width": 960, "jpeg_quality": 85}),
}
JANELA_ESTATISTICA_S = 2.0

# tecla -> (nome da ação, comando RPC do celular, rótulo)
ACOES = {
    pygame.K_t: ("t", "flight.take_off", "Decolar"),
    pygame.K_l: ("l", "flight.land", "Pousar"),
    pygame.K_h: ("h", "home.go_home", "Voltar para casa"),
    # cancela um pouso ou retorno em andamento (nunca a decolagem); o celular escolhe qual
    pygame.K_g: ("g", "flight.cancel_land_or_rth", "Cancelar pouso/retorno"),
}
ACAO_POR_NOME = {nome: (rpc, rotulo) for nome, rpc, rotulo in ACOES.values()}
# Decolar e pousar só valem com o sistema ARMADO (Enter). Voltar para casa e cancelar pouso/retorno
# ficam sempre liberados: são o caminho de segurança.
ACOES_EXIGEM_ARMADO = {"t", "l"}

VERDE, VERMELHO, AMARELO, BRANCO = (80, 220, 100), (240, 80, 70), (250, 200, 60), (235, 235, 235)


class PilotoTecladoMouse(Node):
    def __init__(self) -> None:
        super().__init__("piloto_teclado_mouse")
        self._cmd_pub = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self._gimbal_pub = self.create_publisher(Float32, "/drone/gimbal_pitch_rate", 10)
        self._stick_pub = self.create_publisher(Bool, "/drone/stick_mode", 10)
        self._rpc_pub = self.create_publisher(String, "/drone/rpc/request", 10)
        self.create_subscription(CompressedImage, "/phone/drone_camera/compressed", self._ao_video, 1)
        self.create_subscription(String, "/phone/telemetria", self._ao_telemetria, 10)
        self.create_subscription(String, "/phone/rpc/response", self._ao_resposta_rpc, 10)

        self.piloto = Piloto()
        self.acoes = AcaoSegurada()
        self.video_jpeg: bytes | None = None
        self.video_novo = False
        self._chegadas: collections.deque = collections.deque()  # (instante, bytes) dos últimos quadros
        self.preset_video = ""  # só depois do celular confirmar
        self.telemetria: dict = {}
        self.telemetria_em = 0.0
        self.ultima_resposta = ""
        self.aviso = ""
        self.armado_em = 0.0
        self._rpc_id = 0
        self._rotulos_rpc: dict[int, str] = {}
        self._presets_pendentes: dict[int, str] = {}
        self._zeros_gimbal = 0

    # ----------------------------------------------------------------- recepção

    def _ao_video(self, msg: CompressedImage) -> None:
        self.video_jpeg = bytes(msg.data)
        self.video_novo = True
        self._chegadas.append((time.monotonic(), len(self.video_jpeg)))

    def estatistica_video(self, agora: float) -> tuple[float, float, float]:
        """(quadros/s, KB/quadro, KB/s) recebidos nos últimos `JANELA_ESTATISTICA_S` segundos."""
        while self._chegadas and agora - self._chegadas[0][0] > JANELA_ESTATISTICA_S:
            self._chegadas.popleft()
        n = len(self._chegadas)
        if n == 0:
            return 0.0, 0.0, 0.0
        total = sum(b for _, b in self._chegadas) / 1024
        return n / JANELA_ESTATISTICA_S, total / n, total / JANELA_ESTATISTICA_S

    def _ao_telemetria(self, msg: String) -> None:
        try:
            self.telemetria = json.loads(msg.data)
            self.telemetria_em = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warning("Telemetria inválida do celular.")

    def _ao_resposta_rpc(self, msg: String) -> None:
        try:
            resposta = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        rotulo = self._rotulos_rpc.pop(resposta.get("id"), resposta.get("cmd", "?"))
        preset = self._presets_pendentes.pop(resposta.get("id"), None)
        if preset is not None and resposta.get("ok"):
            self.preset_video = preset
        if resposta.get("ok"):
            self.ultima_resposta = f"{rotulo}: ok"
        else:
            self.ultima_resposta = f"{rotulo}: FALHOU — {resposta.get('error')}"
        self.get_logger().info(self.ultima_resposta)

    # ----------------------------------------------------------------- envio

    def acao_permitida(self, nome: str) -> bool:
        if nome in ACOES_EXIGEM_ARMADO and not self.piloto.armado:
            self.aviso = f"{ACAO_POR_NOME[nome][1]} bloqueado: DESARMADO — Enter para armar antes."
            return False
        return True

    def executar_acao(self, nome: str) -> None:
        if not self.acao_permitida(nome):
            return
        rpc, rotulo = ACAO_POR_NOME[nome]
        self._rpc_id += 1
        self._rotulos_rpc[self._rpc_id] = rotulo
        self.ultima_resposta = f"{rotulo}: enviado..."
        self._rpc_pub.publish(String(data=json.dumps({"id": self._rpc_id, "cmd": rpc})))

    def definir_preset_video(self, tecla: int) -> None:
        rotulo, argumentos = PRESETS_VIDEO[tecla]
        self._rpc_id += 1
        self._rotulos_rpc[self._rpc_id] = f"Vídeo {rotulo}"
        self._presets_pendentes[self._rpc_id] = rotulo
        self.ultima_resposta = f"Vídeo {rotulo}: enviado..."
        self._rpc_pub.publish(String(data=json.dumps(
            {"id": self._rpc_id, "cmd": "video.set_quality", "args": argumentos})))

    def alternar_armado(self) -> None:
        if self.piloto.armado:
            self.desarmar()
        else:
            self.piloto.armar()
            self.armado_em = time.monotonic()
            self.aviso = ""
            self._stick_pub.publish(Bool(data=True))
            self.get_logger().info("ARMADO")

    def desarmar(self) -> None:
        """Zera o movimento e devolve o voo ao controle físico. Seguro de chamar a qualquer hora."""
        estava_armado = self.piloto.armado
        self.piloto.desarmar()
        if estava_armado:
            self._cmd_pub.publish(Twist())
            self._stick_pub.publish(Bool(data=False))
            self.get_logger().info("DESARMADO")

    def sincronizar_com_celular(self, agora: float) -> None:
        """Se o celular saiu do stick mode (botão ASSUMIR CONTROLE, app em segundo plano, drone
        desconectado, falha ao ligar), desarma aqui também. Sem isso o programa continuaria
        "ARMADO" e o próximo Enter desarmaria em vez de armar de novo."""
        if not self.piloto.armado or agora - self.armado_em < GRACA_ARME_S:
            return
        if self.telemetria_em <= self.armado_em or agora - self.telemetria_em > SEM_TELEMETRIA_S:
            return  # sem telemetria recente: o HUD já avisa, e não dá para saber o estado
        if not self.telemetria.get("stick_mode"):
            self.desarmar()
            self.aviso = "Celular saiu do stick mode (ou não conseguiu ligar) — desarmado. Enter para armar de novo."

    def perder_foco(self) -> None:
        self.piloto.soltar_tudo()
        self.acoes.soltar_tudo()
        self.desarmar()

    def publicar(self, c: Comando) -> None:
        if self.piloto.armado:
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = c.x, c.y, c.z
            twist.angular.z = c.yaw
            self._cmd_pub.publish(twist)
        if c.gimbal != 0.0:
            self._zeros_gimbal = ZEROS_AO_PARAR
            self._gimbal_pub.publish(Float32(data=float(c.gimbal)))
        elif self._zeros_gimbal > 0:
            self._zeros_gimbal -= 1
            self._gimbal_pub.publish(Float32(data=0.0))

    def encerrar(self) -> None:
        self.piloto.soltar_tudo()
        self.desarmar()
        for _ in range(ZEROS_AO_PARAR):
            self._gimbal_pub.publish(Float32(data=0.0))


class Janela:
    def __init__(self, no: PilotoTecladoMouse) -> None:
        self.no = no
        pygame.init()
        pygame.display.set_caption("Piloto teclado + mouse")
        self.tela = pygame.display.set_mode((LARGURA, ALTURA), pygame.RESIZABLE)
        self.fonte = pygame.font.Font(None, 26)
        self.fonte_grande = pygame.font.Font(None, 40)
        self.quadro: pygame.Surface | None = None

    # ----------------------------------------------------------------- eventos

    def processar_eventos(self) -> bool:
        """Trata os eventos pendentes; devolve False quando é para sair."""
        no = self.no
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.WINDOWFOCUSLOST:
                no.perder_foco()
                pygame.event.set_grab(False)
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_q and e.mod & pygame.KMOD_CTRL:
                    return False
                if e.key in TECLAS_MOVIMENTO:
                    no.piloto.tecla_pressionada(TECLAS_MOVIMENTO[e.key])
                elif e.key in ACOES:
                    if no.acao_permitida(ACOES[e.key][0]):
                        no.acoes.pressionar(ACOES[e.key][0], time.monotonic())
                elif e.key in PRESETS_VIDEO:
                    no.definir_preset_video(e.key)
                elif e.key == pygame.K_RETURN:
                    no.alternar_armado()
                elif e.key == pygame.K_ESCAPE:
                    no.desarmar()
            elif e.type == pygame.KEYUP:
                if e.key in TECLAS_MOVIMENTO:
                    no.piloto.tecla_solta(TECLAS_MOVIMENTO[e.key])
                elif e.key in ACOES:
                    no.acoes.soltar(ACOES[e.key][0])
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                no.piloto.mouse.pressionar(e.pos)
                pygame.event.set_grab(True)  # arrastar para fora da janela não perde o controle
            elif e.type == pygame.MOUSEMOTION:
                no.piloto.mouse.mover(e.pos)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                no.piloto.mouse.soltar()
                pygame.event.set_grab(False)
        return True

    # ----------------------------------------------------------------- desenho

    def desenhar(self) -> None:
        self.tela.fill((12, 12, 16))
        self._desenhar_video()
        self._desenhar_mouse()
        self._desenhar_hud(time.monotonic())
        pygame.display.flip()

    def _desenhar_video(self) -> None:
        no = self.no
        if no.video_novo and no.video_jpeg is not None:
            no.video_novo = False
            try:
                self.quadro = pygame.image.load(io.BytesIO(no.video_jpeg), "quadro.jpg").convert()
            except pygame.error:
                pass  # quadro corrompido: mantém o anterior
        if self.quadro is None:
            return
        largura, altura = self.tela.get_size()
        escala = min(largura / self.quadro.get_width(), altura / self.quadro.get_height())
        tamanho = (int(self.quadro.get_width() * escala), int(self.quadro.get_height() * escala))
        imagem = pygame.transform.smoothscale(self.quadro, tamanho)
        self.tela.blit(imagem, ((largura - tamanho[0]) // 2, (altura - tamanho[1]) // 2))

    def _desenhar_mouse(self) -> None:
        mouse = self.no.piloto.mouse
        if not mouse.ativo:
            return
        centro = (int(mouse.ancora[0]), int(mouse.ancora[1]))
        # Só um ponto onde você clicou e um fio até o cursor: o quanto você se afasta é o quanto
        # o drone gira / a câmera inclina.
        pygame.draw.line(self.tela, BRANCO, centro, (int(mouse.atual[0]), int(mouse.atual[1])), 1)
        pygame.draw.circle(self.tela, BRANCO, centro, 3)

    def _texto(self, texto: str, pos: tuple[int, int], cor=BRANCO, grande: bool = False) -> int:
        fonte = self.fonte_grande if grande else self.fonte
        sup = fonte.render(texto, True, cor, (0, 0, 0))
        sup.set_alpha(200)
        self.tela.blit(sup, pos)
        return pos[1] + sup.get_height() + 4

    def _desenhar_hud(self, agora: float) -> None:
        no = self.no
        t = no.telemetria
        y = 8
        if no.piloto.armado:
            y = self._texto("ARMADO", (8, y), VERDE, grande=True)
        else:
            y = self._texto("DESARMADO — Enter para armar", (8, y), VERMELHO, grande=True)

        if no.aviso:
            y = self._texto(no.aviso, (8, y), AMARELO)
        if agora - no.telemetria_em > SEM_TELEMETRIA_S:
            y = self._texto("Sem telemetria do celular (app aberto? Wi-Fi? IP do rosbridge?)", (8, y), AMARELO)
        else:
            if not t.get("drone_conectado"):
                y = self._texto("Celular sem ligação com o drone", (8, y), AMARELO)
            if no.piloto.armado and not t.get("stick_mode"):
                y = self._texto("Aguardando o celular ligar o stick mode...", (8, y), AMARELO)
            y = self._texto(self._linha_telemetria(t), (8, y))
        fps, kb_quadro, kb_s = no.estatistica_video(agora)
        preset = no.preset_video or "?"
        y = self._texto(f"Vídeo [{preset}]: {fps:.1f} fps · {kb_quadro:.0f} KB/quadro · {kb_s:.0f} KB/s", (8, y))
        if no.ultima_resposta:
            y = self._texto(no.ultima_resposta, (8, y), AMARELO)
        for nome, fracao in no.acoes.progresso(agora).items():
            y = self._texto(f"Segure: {ACAO_POR_NOME[nome][1]} {'#' * int(fracao * 10):.<10}", (8, y), AMARELO)

        altura = self.tela.get_height()
        self._texto("W A S D mover · Espaço/Shift subir/descer · Setas: ←/→ girar, cima/baixo câmera · arrastar mouse: girar/gimbal · Enter armar · Esc desarmar",
                    (8, altura - 54))
        self._texto("Segurar: T decolar · L pousar · H voltar p/ casa · G cancelar pouso/retorno · "
                    "1-4 qualidade do vídeo · Ctrl+Q sair", (8, altura - 28))

    @staticmethod
    def _linha_telemetria(t: dict) -> str:
        def fmt(chave: str, casas: int = 1, sufixo: str = "") -> str:
            v = t.get(chave)
            return "—" if v is None else f"{v:.{casas}f}{sufixo}"

        voando = "voando" if t.get("voando") else "no solo"
        return (f"Bateria {fmt('bateria_percent', 0, '%')} · Alt {fmt('altitude_m', 1, ' m')} · "
                f"Vel {fmt('velocidade_ms', 1, ' m/s')} · GPS {fmt('satelites', 0)} sat · {voando}")


def main() -> None:
    rclpy.init()
    no = PilotoTecladoMouse()
    janela = Janela(no)
    relogio = pygame.time.Clock()
    ultimo_passo = time.monotonic()
    try:
        while rclpy.ok() and janela.processar_eventos():
            for _ in range(5):
                rclpy.spin_once(no, timeout_sec=0)
            agora = time.monotonic()
            for acao in no.acoes.atualizar(agora):
                no.executar_acao(acao)
            no.sincronizar_com_celular(agora)
            if agora - ultimo_passo >= PERIODO_ENVIO_S:
                no.publicar(no.piloto.passo(agora - ultimo_passo))
                ultimo_passo = agora
            janela.desenhar()
            relogio.tick(60)
    except KeyboardInterrupt:
        pass
    finally:
        no.encerrar()
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()
