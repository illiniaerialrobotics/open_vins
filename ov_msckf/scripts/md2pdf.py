#!/usr/bin/env python3
"""Minimal Markdown -> PDF renderer (reportlab) for the project manuals.
Supports: # / ## / ### headings, paragraphs, **bold**, `code`, fenced code blocks, pipe tables,
'-' bullets, '1.' numbered lists, '---' rules.   usage: md2pdf.py in.md out.pdf [title]"""
import re
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (HRFlowable, KeepTogether, PageBreak, Paragraph, Preformatted,
                                SimpleDocTemplate, Spacer, Table, TableStyle)

FONT, MONO = "Helvetica", "Courier"
try:
    pdfmetrics.registerFont(TTFont("DejaVu", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    pdfmetrics.registerFont(TTFont("DejaVu-Bold", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))
    pdfmetrics.registerFont(TTFont("DejaVuMono", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"))
    from reportlab.pdfbase.pdfmetrics import registerFontFamily
    registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold", italic="DejaVu", boldItalic="DejaVu-Bold")
    FONT, MONO = "DejaVu", "DejaVuMono"
except Exception:
    pass

ss = getSampleStyleSheet()
S = {
    "body": ParagraphStyle("body", parent=ss["Normal"], fontName=FONT, fontSize=9.6, leading=13, spaceAfter=5),
    "h1": ParagraphStyle("h1", parent=ss["Title"], fontName=FONT + ("-Bold" if FONT == "DejaVu" else "-Bold"), fontSize=20, leading=24, alignment=TA_LEFT, spaceAfter=4),
    "h2": ParagraphStyle("h2", parent=ss["Heading2"], fontName=FONT + "-Bold", fontSize=13.5, leading=17, spaceBefore=12, spaceAfter=5, textColor=colors.HexColor("#1a3d6d")),
    "h3": ParagraphStyle("h3", parent=ss["Heading3"], fontName=FONT + "-Bold", fontSize=11, leading=14, spaceBefore=8, spaceAfter=3),
    "code": ParagraphStyle("code", fontName=MONO, fontSize=7.6, leading=9.6, backColor=colors.HexColor("#f2f2f2"),
                           borderColor=colors.HexColor("#c8c8c8"), borderWidth=0.5, borderPadding=4, leftIndent=2, spaceBefore=3, spaceAfter=7),
    "cell": ParagraphStyle("cell", parent=ss["Normal"], fontName=FONT, fontSize=8.4, leading=10.8),
    "cellh": ParagraphStyle("cellh", parent=ss["Normal"], fontName=FONT + "-Bold", fontSize=8.4, leading=10.8),
    "bullet": ParagraphStyle("bullet", parent=ss["Normal"], fontName=FONT, fontSize=9.6, leading=13, leftIndent=14, bulletIndent=4, spaceAfter=2),
}


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def inline(t):
    t = esc(t)
    t = re.sub(r"`([^`]+)`", lambda m: '<font face="%s" size="8.3">%s</font>' % (MONO, m.group(1)), t)
    t = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", t)
    t = re.sub(r"(?<![\w*])\*(?!\s)(.+?)(?<!\s)\*(?![\w*])", r"<i>\1</i>", t)
    return t


def table_flow(rows, width):
    ncol = max(len(r) for r in rows)
    rows = [r + [""] * (ncol - len(r)) for r in rows]
    # column widths proportional to the longest cell, clamped
    lens = [max(len(r[c]) for r in rows) for c in range(ncol)]
    lens = [min(max(l, 8), 60) for l in lens]
    tot = float(sum(lens))
    widths = [width * l / tot for l in lens]
    data = [[Paragraph(inline(c), S["cellh"] if i == 0 else S["cell"]) for c in r] for i, r in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1, hAlign="LEFT")
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#999999")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e6e6e6")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
    ]))
    return [t, Spacer(1, 6)]


def render(md_text, out, title):
    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=16 * mm, bottomMargin=16 * mm, title=title)
    width = A4[0] - 32 * mm
    story, para, code, table = [], [], None, []
    lines = md_text.splitlines()

    def flush_para():
        if para:
            story.append(Paragraph(inline(" ".join(para)), S["body"]))
            para.clear()

    def flush_table():
        if table:
            rows = [[c.strip() for c in r.strip().strip("|").split("|")] for r in table]
            rows = [r for r in rows if not all(re.fullmatch(r":?-{2,}:?", c or "---") for c in r)]
            story.extend(table_flow(rows, width))
            table.clear()

    for ln in lines:
        if code is not None:
            if ln.strip().startswith("```"):
                story.append(Preformatted("\n".join(code), S["code"]))
                code = None
            else:
                code.append(ln)
            continue
        if ln.strip().startswith("```"):
            flush_para(); flush_table(); code = []
            continue
        if ln.strip().startswith("|"):
            flush_para(); table.append(ln); continue
        flush_table()
        s = ln.rstrip()
        if not s.strip():
            flush_para(); continue
        if s.startswith("# "):
            flush_para(); story.append(Paragraph(inline(s[2:]), S["h1"])); continue
        if s.startswith("## "):
            flush_para(); story.append(KeepTogether([Paragraph(inline(s[3:]), S["h2"]), HRFlowable(width="100%", thickness=0.6, color=colors.HexColor("#1a3d6d"), spaceAfter=4)])); continue
        if s.startswith("### "):
            flush_para(); story.append(Paragraph(inline(s[4:]), S["h3"])); continue
        if s.strip() == "---":
            flush_para(); story.append(Spacer(1, 4)); continue
        m = re.match(r"^(\s*)[-*] (.*)", s)
        if m:
            flush_para(); story.append(Paragraph(inline(m.group(2)), S["bullet"], bulletText="•")); continue
        m = re.match(r"^(\d+)\. (.*)", s)
        if m:
            flush_para(); story.append(Paragraph(inline(m.group(2)), S["bullet"], bulletText=m.group(1) + ".")); continue
        para.append(s.strip())
    flush_para(); flush_table()

    def footer(canvas, d):
        canvas.saveState(); canvas.setFont(FONT, 7.5); canvas.setFillColor(colors.grey)
        canvas.drawRightString(A4[0] - 16 * mm, 9 * mm, "%s   p. %d" % (title, d.page)); canvas.restoreState()
    doc.build(story, onFirstPage=footer, onLaterPages=footer)


if __name__ == "__main__":
    src, out = sys.argv[1], sys.argv[2]
    title = sys.argv[3] if len(sys.argv) > 3 else "Manual"
    render(open(src).read(), out, title)
    print("written", out)
