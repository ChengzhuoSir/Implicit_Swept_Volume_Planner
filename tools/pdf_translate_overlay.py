#!/usr/bin/env python3
import math
import re
from pathlib import Path

import fitz
from deep_translator import GoogleTranslator

SRC_PDF = Path('/home/chengzhuo/workspace/plan/src/plan.pdf')
MD_TRANSLATION = Path('/home/chengzhuo/文档/Kimera/Efficient_Swept_Volume_全文翻译与解读.md')
WORK_PDF = Path('/home/chengzhuo/workspace/plan/src/tmp/pdfs/plan_zh_working.pdf')
OUT_PDF = Path('/home/chengzhuo/workspace/plan/src/plan_zh.pdf')
FONT_FILE = '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc'


def normalize_ws(s: str) -> str:
    return re.sub(r'\s+', ' ', s).strip()


def clean_md_text(s: str) -> str:
    s = s.replace('`', '')
    s = s.replace('**', '')
    s = s.replace('$$', '')
    s = s.replace('\u200b', '')
    s = s.replace('\\', '')
    s = re.sub(r'\[(\d+)\]', r'[\1]', s)
    s = normalize_ws(s)
    return s


def load_translation_pool(md_path: Path):
    lines = md_path.read_text(encoding='utf-8').splitlines()

    in_section = False
    skip_note = False
    paras = []
    buf = []

    for raw in lines:
        line = raw.rstrip('\n')
        s = line.strip()

        if s.startswith('## 参考文献'):
            break

        if s.startswith('## '):
            in_section = '（翻译）' in s
            skip_note = False
            if buf:
                paras.append(clean_md_text(' '.join(buf)))
                buf = []
            continue

        if not in_section:
            continue

        if s.startswith('### 我的理解'):
            skip_note = True
            if buf:
                paras.append(clean_md_text(' '.join(buf)))
                buf = []
            continue

        if skip_note:
            continue

        if s == '---' or s.startswith('> 注：'):
            if buf:
                paras.append(clean_md_text(' '.join(buf)))
                buf = []
            continue

        if s.startswith('|'):
            continue

        if not s:
            if buf:
                paras.append(clean_md_text(' '.join(buf)))
                buf = []
            continue

        if s.startswith('- '):
            s = s[2:]

        buf.append(s)

    if buf:
        paras.append(clean_md_text(' '.join(buf)))

    # remove extremely short / non-Chinese noise
    pool = [p for p in paras if len(p) >= 18 and re.search(r'[\u4e00-\u9fff]', p)]
    return pool


def is_reference_block(text: str) -> bool:
    s = normalize_ws(text)
    if not s:
        return True
    if s in ('REFERENCES', 'R EFERENCES'):
        return True
    if re.match(r'^\[\d+\]', s):
        return True
    return False


def is_formula_like(text: str) -> bool:
    s = normalize_ws(text)
    if len(s) < 60 and re.search(r'[=∥∇λβ^_{}]', s):
        letters = len(re.findall(r'[A-Za-z]', s))
        return letters < 20
    return False


def is_long_english_block(text: str) -> bool:
    s = normalize_ws(text)
    if len(s) < 120:
        return False
    if is_reference_block(s):
        return False
    if is_formula_like(s):
        return False
    letters = len(re.findall(r'[A-Za-z]', s))
    return letters >= 50


def heading_or_caption_map(text: str):
    s = normalize_ws(text)
    mapped = {
        'Efficient Swept Volume-Based Trajectory Generation for Arbitrary-Shaped Ground Robot Navigation': '基于高效扫掠体积的任意形状地面机器人轨迹生成',
        'Abstract—': '摘要——',
        'I. INTRODUCTION': 'I. 引言',
        'II. RELATED WORKS': 'II. 相关工作',
        'III. FRAMEWORK OVERVIEW': 'III. 框架总览',
        'IV. TOPOLOGICAL PATH GENERATION': 'IV. 拓扑路径生成',
        'V. SE(2) MOTION SEQUENCE GENERATION': 'V. SE(2) 运动序列生成',
        'VI. TRAJECTORY OPTIMIZATION': 'VI. 轨迹优化',
        'A. Trajectory Generation Strategy': 'A. 轨迹生成策略',
        'B. Trajectory Formulation': 'B. 轨迹形式化',
        'VII. EXPERIMENTS': 'VII. 实验',
        'A. Benchmark Comparison': 'A. 基准对比',
        'B. Real-world Tests': 'B. 真实世界测试',
        'VIII. CONCLUSION AND FUTURE WORK': 'VIII. 结论与未来工作',
        'Algorithm 1: Path Shortcut': '算法 1：路径捷径化',
        'Algorithm 2: SE(2) Motion Sequence Generation': '算法 2：SE(2) 运动序列生成',
        'TABLE I: Real-world Experiment': '表 I：真实世界实验',
        'arXiv:2504.07554v2 [cs.RO] 6 Jan 2026': 'arXiv:2504.07554v2 [cs.RO] 2026年1月6日',
        '1) Baselines:': '1）对比方法：我们将所提方法与两种任意形状机器人运动规划基线（SVSDF 与 RC-ESDF）进行比较。',
        '2) Settings:': '2）实验设置：在 Office 与 Maze 两类环境中，使用 L 型与 T 型非凸机器人进行评测。',
        '3) Results:': '3）结果：本文方法在保证连续无碰撞的同时，兼顾更高成功率与更低计算开销。',
        "the method’s efficiency.": '该方法的效率。',
        "the method's efficiency.": '该方法的效率。',
    }

    for k, v in mapped.items():
        if k in s:
            return v

    if s.startswith('Fig. 1:'):
        return '图 1：真实实验中，T 型配送机器人在杂乱室内环境中的导航过程。所提框架生成整机轨迹以保证精确连续避碰。'
    if s.startswith('Fig. 2:'):
        return '图 2：所提规划框架总览。'
    if s.startswith('Fig. 3:'):
        return '图 3：（a）实验机器人及其内切圆；（b）机器人几何内部构建的机体系 ESDF，梯度和可用于避障。'
    if s.startswith('Fig. 4:'):
        return '图 4：基于机器人真实几何对绕行长路径进行缩短，得到更平滑的拓扑简化路径。'
    if s.startswith('Fig. 5:'):
        return '图 5：T 型机器人与环境碰撞检测示例，机器人核离散为 18 个朝向（每次旋转 20°）。'
    if s.startswith('Fig. 6:'):
        return '图 6：碰撞风险较高的 SE(2) 运动序列细化流程：先局部推离，再对子段递归调整。'
    if s.startswith('Fig. 7:'):
        return '图 7：T 型机器人在障碍密集区域的两条 SE(2) 细化序列：绿色为成功优化段，红色表示失败段。'
    if s.startswith('Fig. 8:'):
        return '图 8：迷宫场景下，本文方法与两种任意形状轨迹优化方法的仿真对比。'
    if s.startswith('Fig. 9:'):
        return '图 9：Office 地图中三种方法生成的两条拓扑轨迹。'
    if s.startswith('Fig. 10:'):
        return '图 10：两地图、两机器人形状、两条代表路径下的耗时与连续无碰撞成功率对比。'
    if s.startswith('Fig. 11:'):
        return '图 11：鸟瞰图显示 T 型机器人在狭窄、障碍密集且柜体不规则摆放的空间中安全穿行。'

    # author block intentionally stays in English names
    return None


def estimate_fontsize(rect: fitz.Rect, text: str, start: float) -> float:
    fs = start
    compact = text.replace('\n', '')
    while fs >= 4.8:
        chars_per_line = max(4, int(rect.width / (fs * 0.95)))
        lines = max(1, math.ceil(len(compact) / chars_per_line))
        need_h = lines * fs * 1.22
        if need_h <= rect.height * 0.98:
            return fs
        fs -= 0.3
    return 4.8


def pick_start_font(text: str, rect: fitz.Rect) -> float:
    s = normalize_ws(text)
    if 'Efficient Swept Volume-Based Trajectory Generation' in s:
        return 14.5
    if re.match(r'^[IVX]+\.', s) or s.startswith('A.') or s.startswith('B.'):
        return 9.2
    if s.startswith('Fig.') or s.startswith('TABLE') or s.startswith('Algorithm'):
        return 7.4
    if rect.height >= 260:
        return 8.0
    if rect.height >= 120:
        return 8.2
    return 7.8


def draw_replace(page, rect: fitz.Rect, text: str):
    wipe = fitz.Rect(rect.x0 - 0.8, rect.y0 - 0.2, rect.x1 + 0.8, rect.y1 + 0.2)
    page.draw_rect(wipe, color=(1, 1, 1), fill=(1, 1, 1), overlay=True)
    start_fs = pick_start_font(text, rect)
    fs = estimate_fontsize(rect, text, start_fs)
    page.insert_textbox(
        rect,
        text,
        fontsize=fs,
        fontname='noto-cjk',
        fontfile=FONT_FILE,
        lineheight=1.15,
        color=(0, 0, 0),
        align=0,
        overlay=True,
    )


def should_skip_block(text: str) -> bool:
    s = normalize_ws(text)
    if not s:
        return True
    if is_reference_block(s):
        return True
    # skip tiny labels in schematic diagrams
    if len(s) < 20 and not re.search(r'^(I\.|II\.|III\.|IV\.|V\.|VI\.|VII\.|VIII\.|A\.|B\.|Fig\.|Algorithm|TABLE)', s):
        return True
    return False


def prepare_source_text(text: str) -> str:
    s = text.replace('\n', ' ')
    # Merge line-break hyphenation artifacts.
    s = re.sub(r'([A-Za-z])-\s+([A-Za-z])', r'\1\2', s)
    s = normalize_ws(s)
    return s


def should_translate_by_api(page_num: int, rect: fitz.Rect, text: str) -> bool:
    s = normalize_ws(text)
    if is_reference_block(s):
        return False
    if is_formula_like(s):
        return False
    # Keep author names untouched.
    if page_num == 1 and len(s) < 220 and ',' in s and re.search(r'\b(Yisheng|Longji|Yixi|Jianheng|Fangcheng|Mingpu|Siqi|Haotian|Fu)\b', s):
        return False
    # Avoid tiny labels in figures / flowcharts.
    if rect.height < 20 and len(s) < 45:
        return False
    letters = len(re.findall(r'[A-Za-z]', s))
    return letters >= 20


def translate_text_google(translator, cache, text: str) -> str:
    if text in cache:
        return cache[text]
    # Safe chunking for long blocks.
    max_chunk = 3500
    if len(text) <= max_chunk:
        out = translator.translate(text)
    else:
        parts = []
        i = 0
        while i < len(text):
            j = min(len(text), i + max_chunk)
            if j < len(text):
                cut = text.rfind('. ', i, j)
                if cut > i + 200:
                    j = cut + 1
            parts.append(text[i:j].strip())
            i = j
        out = ' '.join(translator.translate(p) for p in parts if p)
    out = normalize_ws(out)
    cache[text] = out
    return out


def main():
    assert SRC_PDF.exists(), f'missing {SRC_PDF}'
    assert MD_TRANSLATION.exists(), f'missing {MD_TRANSLATION}'
    WORK_PDF.parent.mkdir(parents=True, exist_ok=True)

    _ = load_translation_pool(MD_TRANSLATION)  # Keep translation asset dependency explicit.
    translator = GoogleTranslator(source='en', target='zh-CN')
    tcache = {}

    doc = fitz.open(SRC_PDF)

    replaced = 0
    seq_replaced = 0

    for page_num, page in enumerate(doc, start=1):
        blocks = sorted(page.get_text('blocks'), key=lambda b: (b[1], b[0]))
        for b in blocks:
            rect = fitz.Rect(b[:4])
            raw = (b[4] or '').strip()
            if rect.width < 85 and rect.height > 80:
                # Side metadata / rotated margin text - keep original.
                continue
            if should_skip_block(raw):
                continue

            # keep bibliography entries in English
            if page_num >= 8 and re.match(r'^\[\d+\]', normalize_ws(raw)):
                continue

            mapped = heading_or_caption_map(raw)
            replacement = None

            if mapped:
                replacement = mapped
            elif should_translate_by_api(page_num, rect, raw):
                src = prepare_source_text(raw)
                try:
                    replacement = translate_text_google(translator, tcache, src)
                    seq_replaced += 1
                except Exception:
                    replacement = None

            if not replacement:
                continue

            draw_replace(page, rect, replacement)
            replaced += 1

    doc.save(WORK_PDF)
    doc.close()

    # copy working as final
    fitz.open(WORK_PDF).save(OUT_PDF)

    print(f'replaced_blocks={replaced}')
    print(f'api_translated_blocks={seq_replaced}')
    print(f'working_pdf={WORK_PDF}')
    print(f'output_pdf={OUT_PDF}')


if __name__ == '__main__':
    main()
