# plan.pdf Chinese Overlay Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Generate `plan_zh.pdf` by translating English body text into Chinese while preserving original PDF layout as much as possible, keeping references in English.

**Architecture:** Build a block-level PDF overlay pipeline: extract text blocks from original PDF, selectively redact body blocks, insert Chinese replacements in-place with adaptive font sizing/line spacing, then run page render validation and manual coordinate tuning for problematic blocks.

**Tech Stack:** Python 3, PyMuPDF (fitz), pypdf, Poppler (`pdftoppm`), Noto CJK fonts.

---

### Task 1: Environment and Input Validation

**Files:**
- Modify: `docs/plans/2026-03-05-plan-pdf-zh-implementation.md`
- Input: `plan.pdf`

**Step 1: Verify Python dependencies**
Run: `python3 -c "import fitz,pypdf;print('ok')"`
Expected: prints `ok`

**Step 2: Verify source file and output directories**
Run: `ls -la plan.pdf docs/plans`
Expected: file and directory exist

### Task 2: Implement Overlay Translator Script

**Files:**
- Create: `tools/pdf_translate_overlay.py`
- Input: `plan.pdf`
- Input: `../../文档/Kimera/Efficient_Swept_Volume_全文翻译与解读.md` (translation source)

**Step 1: Build block extraction and filtering**
- Extract page text blocks with coordinates.
- Skip blocks that are figures-only, formulas-only, or references section.

**Step 2: Build text replacement strategy**
- Sequentially map translated Chinese paragraphs to eligible English blocks.
- Keep title/authors/section headers in Chinese where available.

**Step 3: Render replacement into same block boxes**
- Redact source text area to white.
- Insert Chinese text via `insert_textbox` with adaptive font sizing.

### Task 3: Generate First-pass PDF and Visual QA

**Files:**
- Create: `tmp/pdfs/plan_zh_working.pdf`
- Create: `tmp/pdfs/plan_zh_pages/*.png`

**Step 1: Run script**
Run: `python3 tools/pdf_translate_overlay.py`
Expected: outputs a working pdf

**Step 2: Render pages**
Run: `pdftoppm -png tmp/pdfs/plan_zh_working.pdf tmp/pdfs/plan_zh_pages/plan_zh`
Expected: one PNG per page

**Step 3: Inspect and collect overflow blocks**
- Identify clipped/overlapped blocks and record page/rect coordinates.

### Task 4: Manual Correction Pass

**Files:**
- Modify: `tools/pdf_translate_overlay.py`
- Output: `plan_zh.pdf`

**Step 1: Add per-block tuning table**
- Add optional per-page overrides for font size and line height.

**Step 2: Re-run generation and QA until clean**
Run:
- `python3 tools/pdf_translate_overlay.py`
- `pdftoppm -png plan_zh.pdf tmp/pdfs/plan_zh_pages/plan_zh`
Expected: no obvious text clipping/overlap in body text

### Task 5: Delivery

**Files:**
- Output: `plan_zh.pdf`
- Notes: `docs/plans/2026-03-05-plan-pdf-zh-design.md`

**Step 1: Confirm acceptance criteria**
- page count unchanged
- references in English
- body largely Chinese
- layout close to original

**Step 2: Report outputs and known residual issues**
- Provide file path and any pages with minor compromises.
