import base64, re, sys
P, out = sys.argv[1], sys.argv[2]
html = open(f"{P}/arquitectura.tpl.html", encoding="utf-8").read()
def sub(m):
    return "data:image/jpeg;base64," + base64.b64encode(open(f"{P}/img/{m.group(1)}.jpg", "rb").read()).decode()
html, n = re.subn(r"@@(\w+)", sub, html)
open(out, "w", encoding="utf-8").write(html); print(n, "imágenes,", len(html) // 1024, "KB")
