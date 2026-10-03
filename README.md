# controle_drone_teclado

Pilota um **DJI Mini SE** pelo teclado e mouse do computador, vendo o vídeo do drone na mesma
janela. O app Android (`app_proprio`, branch `teclado-mouse`) é só a ponte: recebe os comandos
por ROS2/rosbridge e repassa ao DJI SDK. Os tópicos e comandos estão no `PROTOCOLO.md` daquele
projeto.

```
teclado/mouse → controle.py → piloto_teclado_mouse (ROS2) → rosbridge (WebSocket :9090)
                                                                   │  Wi-Fi (hotspot do PC)
                                                         app_proprio (celular) → controle DJI → drone
```

## Setup e execução

Pré-requisitos: ROS2 Humble e `ros-humble-rosbridge-suite` instalados (apt).

```bash
./setup_ambiente.sh          # uma vez: venv com pygame e pytest
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 -m colcon build --symlink-install
source install/setup.bash
ros2 launch ponte_teclado_drone sistema.launch.py
```

No celular, informe o IP do computador e a porta `9090` (toque na linha de status do app). Com o
hotspot do GNOME o computador costuma ser `10.42.0.1`; confira com `ip addr`.

Testes (sem ROS2 e sem drone): `cd src/ponte_teclado_drone && python -m pytest test`.

## Controles (janela em foco)

| Entrada | Ação |
|---|---|
| `W` `A` `S` `D` | Frente, esquerda, trás, direita (plano paralelo ao solo) |
| `Espaço` / `Shift` | Subir / descer |
| Arrastar o mouse (botão esquerdo) | Como puxar a imagem. Arrastar para a direita: o drone gira para a esquerda (a imagem vai para a direita). Arrastar para baixo: a câmera sobe (a imagem desce). Vice-versa nos outros sentidos |
| `Enter` | Arma / desarma o envio de comandos de voo |
| `Esc` | Desarma na hora (o controle físico da DJI volta a mandar) |
| Segurar `T` / `L` | Decolar / pousar (só com o sistema **armado**; desarmado a tecla é ignorada e a janela avisa) |
| Segurar `H` / `G` | Voltar para casa / cancelar um pouso ou retorno em andamento (não cancela a decolagem) |
| `←` / `→` | Gira o drone em torno do próprio eixo, para a esquerda / direita, enquanto apertada (só armado). Soma com o arrasto horizontal do mouse |
| `↑` / `↓` | Inclina a câmera (gimbal) para cima / para baixo, enquanto apertada. Soma com o mouse e funciona mesmo desarmado |
| `1` `2` `3` `4` | Qualidade do vídeo: Desempenho (12 fps, 480 px) / Equilibrado (8 fps, 640 px, o padrão) / Qualidade (8 fps, 960 px) / Máximo (20 fps, 960 px, qualidade 85) |
| `Ctrl+Q` | Sair |

Segurar = manter a tecla apertada por ~0,8 s; a janela mostra o progresso. Isso evita decolar,
pousar ou iniciar o retorno por um toque acidental.

O mouse funciona como um joystick: o ponto do clique é o centro, o deslocamento dali vira
velocidade (zona morta de 10 px, máximo em 150 px, curva exponencial, filtro suave) e soltar o
botão zera tudo. Na tela aparece só um ponto onde você clicou e um fio até o cursor. O gimbal mantém o ângulo quando você solta. Os números ficam no topo de
`controle.py`.

A janela mostra o preset ativo (só depois do celular confirmar) e os quadros por segundo, KB por quadro e
KB/s realmente recebidos. Se a imagem engasgar, desça de preset: os de maior qualidade pesam várias vezes
mais no Wi-Fi e na CPU do celular. Isso só reduz o tráfego do vídeo reenviado ao PC, não o link do drone.
Os valores dos presets são um primeiro palpite (`PRESETS_VIDEO` em `piloto_teclado_mouse.py`).

## Segurança

- **Desarmado por padrão.** Nenhum comando de voo sai até apertar `Enter` — isso inclui decolar e pousar (`T`/`L`). Voltar para casa (`H`/`G`) funciona sempre. O gimbal funciona
  desarmado (a câmera não faz parte do voo).
- O celular zera os sticks se ficar 500 ms sem comando (Wi-Fi caiu, programa travou) e o drone paira.
- Perder o foco da janela, fechar ou `Ctrl+C` desarma, zera e devolve o voo ao controle físico.
- O **botão de pânico existe só no celular** (de propósito). Mantenha o controle físico à mão.
- A janela avisa quando o celular não está em stick mode, sem telemetria ou sem ligação com o drone.

## Limitações do Mini SE (lembrar ao ampliar este programa)

- No gimbal do Mini SE só a **inclinação (pitch)** é controlável: −90° a 0°, ou até +20° com
  `gimbal.set_pitch_range_extension`. Giro e rolagem são automáticos. Este programa só usa pitch.
- Nem todo comando do `PROTOCOLO.md` é garantido no Mini SE; `system.probe` mostra o que o drone
  respondeu de fato.

**Nada disso foi testado com o drone real** — só com mensagens sintéticas. Veja a ordem de testes
sugerida no `CLAUDE.md` do `app_proprio`.
