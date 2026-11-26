"""Build one offline HTML file from the slide deck sources in ../deck/project:
every slide (scaled down) with its speaker notes beside it.

    python tools/build_deck_html.py        -> deck/ICP_Node_Walkthrough.html

The deck itself lives at https://claude.ai/artifact/L8ahk2hNVJVEC5VsPUUStM (Slides type,
which also exports PowerPoint and PDF); this file is the copy that works without it.
Fonts come from Google Fonts when online, otherwise the fallback fonts are used.
"""
import html
import json
import re
from pathlib import Path

DECK = Path(__file__).resolve().parent.parent / "deck"
OUT = DECK / "ICP_Node_Walkthrough.html"

deck = json.loads((DECK / "project" / "deck.json").read_text(encoding="utf-8"))
css = (DECK / "project" / "deck.css").read_text(encoding="utf-8")
fonts = "".join(f'<link rel="stylesheet" href="{f["href"]}">' for f in deck["faces"].values() if "href" in f)

page_css = """
* { margin: 0 }
html, body { background: #0B1015; color: #E8EEF2; font-family: 'Inter', Arial, sans-serif; }
.top { padding: 24px 32px; display: flex; flex-direction: column; gap: 6px; }
.top h1 { font: 700 28px/1.2 'Rubik', Arial, sans-serif; }
.top p { font-size: 15px; color: #94A4B1; }
.pair { display: grid; grid-template-columns: 960px minmax(280px, 1fr); gap: 24px; padding: 16px 32px; align-items: start; }
.frame { width: 960px; height: 540px; overflow: hidden; border-radius: 8px; box-shadow: 0 6px 24px rgba(0,0,0,.4); }
.frame > section { width: 1920px !important; height: 1080px !important; transform: scale(.5); transform-origin: 0 0;
  overflow: hidden !important; box-sizing: border-box !important; position: relative; }
.notes { font-size: 16px; line-height: 1.6; color: #C3CED6; background: #151E26; border-radius: 8px; padding: 16px 20px; }
.notes b { display: block; color: #FF8B4F; font-size: 12px; letter-spacing: .12em; text-transform: uppercase; margin-bottom: 6px; }
.part { padding: 28px 32px 4px; color: #FF8B4F; font: 600 13px/1 'Inter', Arial, sans-serif; letter-spacing: .14em; text-transform: uppercase; }
aside { display: none !important; }
.fragment { opacity: 1 !important; }
table { width: 100%; border-collapse: separate; border-spacing: 0; }
th, td { padding: .35em .6em; text-align: left; border-bottom: 2px solid #ffffff22; }
@media (max-width: 1300px) { .pair { grid-template-columns: 1fr; } }
"""

starts = {v["start"]: v["description"] for v in deck.get("sections", {}).values()}
body = []
for i, sid in enumerate(deck["order"], 1):
    src = (DECK / "project" / "slides" / f"{sid}.html").read_text(encoding="utf-8")
    src = re.sub(r'src="/_blob/[^"]+"', 'src="rviz_end.jpg"', src)
    m = re.search(r"<aside>(.*?)</aside>", src, re.S)
    notes = m.group(1).strip() if m else ""
    if sid in starts:
        body.append(f'<p class="part">{html.escape(starts[sid])}</p>')
    body.append(f'<div class="pair" id="{sid}"><div class="frame">{src}</div>'
                f'<div class="notes"><b>Slide {i} · speaker notes</b>{notes}</div></div>')

OUT.write_text(
    "<!doctype html><html lang=en><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width, initial-scale=1'>"
    f"<title>{html.escape(deck['title'])}</title>{fonts}<style>{css}\n{page_css}</style></head><body>"
    f"<div class=top><h1>{html.escape(deck['title'])}</h1>"
    f"<p>{len(deck['order'])} slides with speaker notes. Offline copy of the Slides deck; open explorer.html for the hover explanations.</p></div>"
    + "\n".join(body) + "</body></html>",
    encoding="utf-8")
print(OUT, len(deck["order"]), "slides")
