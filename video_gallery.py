"""Reproductores locales para inspeccionar las secuencias ya exportadas."""
import base64
from pathlib import Path
from main import SEQUENCES


def gallery(root=Path('.'), embed=False):
    cards = []
    for index, name in enumerate(SEQUENCES):
        path = root/'results'/'videos'/f'{name}.mp4'
        src = ('data:video/mp4;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')
               if embed else f'results/videos/{name}.mp4')
        vid = f'lidar-video-{index}'
        cards.append(f'''<section style="margin:24px 0">
<h3>{name}</h3>
<video id="{vid}" controls preload="metadata" playsinline style="width:100%;max-width:1400px" src="{src}"></video>
<div style="display:flex;gap:12px;align-items:center;margin:8px 0">
<button onclick="stepFrame('{vid}',-1)">← Fotograma</button>
<button onclick="stepFrame('{vid}',1)">Fotograma →</button>
<label>Velocidad <select onchange="document.getElementById('{vid}').playbackRate=Number(this.value)">
<option value="0.25">0,25×</option><option value="0.5">0,5×</option><option value="1" selected>1×</option></select></label>
</div></section>''')
    return '''<div style="font-family:Arial,sans-serif">
<p>200 fotogramas por secuencia · 10 fps · 20 segundos · sin tracking.</p>
<p>Izquierda: nube LiDAR. Derecha: cajas y conteo estimado del instante.
Puedes pausar, cambiar la velocidad y ampliar a pantalla completa.</p>
'''+''.join(cards)+'''</div><script>
function stepFrame(id, direction) {
 const v=document.getElementById(id); v.pause();
 const frame=Math.floor(v.currentTime*10+0.001);
 v.currentTime=(Math.max(0,Math.min(199,frame+direction))+0.01)/10;
}
</script>'''


if __name__ == '__main__':
    body=gallery()
    Path('Ver_videos.html').write_text('''<!doctype html><html lang="es"><meta charset="utf-8">
<title>UrbanIng-V2X · Vídeos</title><style>
body{background:#09101a;color:#e2ebf3;max-width:1500px;margin:32px auto;padding:0 24px;font-family:Arial}
button,select{padding:9px 14px;font-size:16px;cursor:pointer}h1{font-size:30px}
</style><h1>Vehículos en la intersección</h1>'''+body+'</html>',encoding='utf-8')
