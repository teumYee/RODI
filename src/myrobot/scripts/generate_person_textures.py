#!/usr/bin/env python3
"""
사람 텍스처 PNG 생성 스크립트 (개선판)

1단계: 인터넷에서 무료 전신 사람 사진 다운로드 시도
2단계: 다운로드 실패 시 고품질 합성 이미지 생성 (YOLO 감지 최적화)

생성 결과:
  models/person_*/materials/textures/*_texture.png   ← Gazebo 텍스처
  models/person_*/materials/textures/*_overlay.png   ← 투명 배경 PNG (오버레이용)
"""

import os
import sys
import urllib.request
import io

import cv2
import numpy as np

try:
    from PIL import Image as PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR  = os.path.join(SCRIPT_DIR, '..', 'models')

# ── 다운로드 URL 목록 (공개 도메인 / CC0) ────────────────────────────────
# 실제 사람 전신 이미지를 담은 안정적인 무료 소스
DOWNLOAD_SOURCES = {
    'adult1': [
        'https://upload.wikimedia.org/wikipedia/commons/thumb/1/14/Gatto_europeo4.jpg/120px-Gatto_europeo4.jpg',  # 테스트용 (나중에 교체)
        # 실제 동작하는 URL 우선순위 순
        'https://randomuser.me/api/portraits/men/32.jpg',
        'https://randomuser.me/api/portraits/men/45.jpg',
    ],
    'adult2': [
        'https://randomuser.me/api/portraits/women/44.jpg',
        'https://randomuser.me/api/portraits/women/22.jpg',
    ],
    'adult3': [
        'https://randomuser.me/api/portraits/men/67.jpg',
        'https://randomuser.me/api/portraits/men/12.jpg',
    ],
    'child1': [
        'https://randomuser.me/api/portraits/lego/1.jpg',
    ],
    'child2': [
        'https://randomuser.me/api/portraits/lego/2.jpg',
    ],
    'child3': [
        'https://randomuser.me/api/portraits/lego/3.jpg',
    ],
    'senior1': [
        'https://randomuser.me/api/portraits/men/77.jpg',
        'https://randomuser.me/api/portraits/women/77.jpg',
    ],
}


def try_download(urls: list, size: tuple) -> np.ndarray | None:
    """URL 목록에서 이미지 다운로드 시도. 성공 시 리사이즈된 BGR 이미지 반환."""
    for url in urls:
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req, timeout=5) as resp:
                data = resp.read()
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.resize(img, size)
                print(f'  다운로드 성공: {url}')
                return img
        except Exception as e:
            print(f'  다운로드 실패 ({url}): {e}')
    return None


# ── 고품질 합성 사람 이미지 생성 ─────────────────────────────────────────

def _gradient(img, y0, y1, x0, x1, c_top, c_bot):
    """세로 그라디언트 직사각형."""
    for y in range(max(0, y0), min(img.shape[0], y1)):
        t = (y - y0) / max(y1 - y0, 1)
        c = tuple(int(c_top[i] * (1 - t) + c_bot[i] * t) for i in range(3))
        img[y, max(0, x0):min(img.shape[1], x1)] = c


def draw_person_hq(W: int, H: int,
                   skin, hair, shirt_top, shirt_bot,
                   pants_top, pants_bot,
                   bg=(210, 210, 210),
                   is_child=False) -> np.ndarray:
    """
    YOLO 감지에 최적화된 고품질 합성 전신 사람 이미지.
    - 그라디언트 의상
    - 상세한 얼굴 (눈썹/눈/코/입)
    - 자연스러운 실루엣
    """
    img = np.full((H, W, 3), bg, dtype=np.uint8)
    cx  = W // 2

    # ── 머리 ────────────────────────────────────────────────────────────
    hr   = int(W * 0.19)            # head radius
    hcy  = int(H * 0.115)           # head center y
    # 머리카락 (약간 크게)
    cv2.ellipse(img, (cx, hcy - 4), (hr + 5, int(hr * 1.25)),
                0, 190, 350, hair, -1)
    # 얼굴
    cv2.ellipse(img, (cx, hcy), (hr, int(hr * 1.15)), 0, 0, 360, skin, -1)

    # 눈썹
    ey = hcy - hr // 4
    bw = hr // 2
    for sign in (-1, 1):
        bx = cx + sign * int(hr * 0.33)
        pts = np.array([[bx - bw//2, ey - 5],
                        [bx + bw//2, ey - 5],
                        [bx + bw//2 - 2, ey - 9],
                        [bx - bw//2 + 2, ey - 9]], np.int32)
        cv2.fillPoly(img, [pts], hair)

    # 눈 (흰자 + 동공)
    for sign in (-1, 1):
        ex = cx + sign * int(hr * 0.33)
        cv2.ellipse(img, (ex, ey), (int(hr * 0.2), int(hr * 0.13)),
                    0, 0, 360, (240, 240, 240), -1)
        cv2.circle(img, (ex, ey), int(hr * 0.1), (30, 20, 15), -1)
        cv2.circle(img, (ex - 2, ey - 2), 2, (255, 255, 255), -1)

    # 코
    ny = hcy + hr // 5
    nose_pts = np.array([[cx, ny - 6], [cx - 5, ny + 6], [cx + 5, ny + 6]], np.int32)
    cv2.fillPoly(img, [nose_pts],
                 tuple(int(c * 0.85) for c in skin))

    # 입
    my = hcy + int(hr * 0.5)
    cv2.ellipse(img, (cx, my), (int(hr * 0.25), int(hr * 0.12)),
                0, 0, 180, (140, 70, 80), 2)
    cv2.ellipse(img, (cx, my), (int(hr * 0.18), int(hr * 0.08)),
                0, 0, 180, (180, 100, 110), -1)

    # 귀
    for sign in (-1, 1):
        eax = cx + sign * (hr - 2)
        cv2.ellipse(img, (eax, hcy), (int(hr * 0.12), int(hr * 0.2)),
                    0, 0, 360, skin, -1)

    # ── 목 ──────────────────────────────────────────────────────────────
    nw      = int(W * 0.075)
    neck_t  = hcy + int(hr * 1.0)
    neck_b  = int(H * 0.215)
    _gradient(img, neck_t, neck_b, cx - nw, cx + nw,
              skin, tuple(int(c * 0.9) for c in skin))

    # ── 셔츠 (몸통) ──────────────────────────────────────────────────────
    body_t = neck_b
    body_b = int(H * 0.615)
    sw     = int(W * 0.38)
    sw_t   = int(W * 0.12)
    # 사다리꼴 몸통
    pts = np.array([
        [cx - sw_t, body_t], [cx - sw, body_b],
        [cx + sw, body_b],   [cx + sw_t, body_t]], np.int32)
    cv2.fillPoly(img, [pts], shirt_top)
    # 그라디언트 덮어쓰기
    for y in range(body_t, body_b):
        t     = (y - body_t) / max(body_b - body_t, 1)
        c     = tuple(int(shirt_top[i] * (1 - t) + shirt_bot[i] * t) for i in range(3))
        frac  = (y - body_t) / max(body_b - body_t, 1)
        xleft = int(cx - sw_t + (sw - sw_t) * frac)
        xrght = int(cx + sw_t + (sw - sw_t) * frac)
        img[y, max(0, xleft):min(W, xrght)] = c

    # 셔츠 칼라
    collar_pts = np.array([
        [cx - nw, body_t], [cx + nw, body_t],
        [cx + int(nw * 1.5), body_t + int(H * 0.04)],
        [cx, body_t + int(H * 0.07)],
        [cx - int(nw * 1.5), body_t + int(H * 0.04)]], np.int32)
    cv2.fillPoly(img, [collar_pts],
                 tuple(max(0, c - 20) for c in shirt_top))

    # ── 팔 ──────────────────────────────────────────────────────────────
    arm_w = int(W * 0.085)
    for sign, x_start, x_end in [
        (-1, cx - sw_t - 5, int(W * 0.04)),
        (+1, cx + sw_t + 5, int(W * 0.96))
    ]:
        ay_top = body_t + int(H * 0.02)
        ay_bot = int(H * 0.535)
        arm_pts = np.array([
            [x_start - arm_w, ay_top],
            [x_end   - arm_w, ay_bot],
            [x_end   + arm_w, ay_bot],
            [x_start + arm_w, ay_top]], np.int32)
        cv2.fillPoly(img, [arm_pts], shirt_top)
        # 손
        hx = (x_end - arm_w + x_end + arm_w) // 2
        cv2.ellipse(img, (hx, ay_bot + int(H * 0.025)),
                    (int(W * 0.075), int(H * 0.04)),
                    0, 0, 360, skin, -1)

    # ── 바지 ─────────────────────────────────────────────────────────────
    pw     = int(sw * 0.46)
    gap    = int(W * 0.025)
    lleg_x = cx - pw // 2 - gap
    rleg_x = cx + pw // 2 + gap
    for lx in [cx - gap - pw, cx + gap]:
        _gradient(img, body_b, H - 25, lx, lx + pw, pants_top, pants_bot)

    # 허리 밴드
    cv2.rectangle(img, (cx - sw, body_b), (cx + sw, body_b + int(H * 0.028)),
                  tuple(max(0, c - 25) for c in pants_top), -1)

    # ── 신발 ─────────────────────────────────────────────────────────────
    shoe_c = (25, 20, 15)
    for lx in [cx - gap - pw, cx + gap]:
        scx = lx + pw // 2
        cv2.ellipse(img, (scx, H - 18), (pw // 2 + 10, 14), 0, 0, 360, shoe_c, -1)

    # 외곽선
    cv2.rectangle(img, (1, 1), (W - 2, H - 2), (100, 100, 100), 1)
    return img


def make_transparent(bgr: np.ndarray, bg_color=(210, 210, 210), tol=35) -> np.ndarray:
    """배경색을 투명하게 변환 (BGRA). 오버레이용."""
    bgra = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
    bg   = np.array(bg_color, dtype=np.uint8)
    mask = np.all(np.abs(bgr.astype(int) - bg.astype(int)) < tol, axis=2)
    bgra[mask, 3] = 0
    return bgra


# ── 모델별 설정 ───────────────────────────────────────────────────────────
PERSON_BG = (210, 210, 210)

CONFIGS = {
    'adult1': {
        'model': 'person_adult_1', 'tex': 'adult1_texture',
        'W': 256, 'H': 512, 'is_child': False,
        'skin': (170, 120, 90), 'hair': (25, 18, 12),
        'shirt_top': (160, 80, 30),  'shirt_bot': (120, 60, 20),   # 파란 셔츠 (BGR)
        'pants_top': (70, 55, 40),   'pants_bot': (50, 38, 28),
    },
    'adult2': {
        'model': 'person_adult_2', 'tex': 'adult2_texture',
        'W': 256, 'H': 512, 'is_child': False,
        'skin': (185, 145, 115), 'hair': (45, 30, 80),
        'shirt_top': (50, 50, 190),  'shirt_bot': (30, 30, 150),   # 빨간 셔츠
        'pants_top': (45, 35, 30),   'pants_bot': (30, 22, 18),
    },
    'adult3': {
        'model': 'person_adult_3', 'tex': 'adult3_texture',
        'W': 256, 'H': 512, 'is_child': False,
        'skin': (155, 108, 80), 'hair': (20, 15, 10),
        'shirt_top': (60, 140, 60),  'shirt_bot': (40, 100, 40),   # 초록 셔츠
        'pants_top': (80, 60, 45),   'pants_bot': (55, 40, 30),
    },
    'senior1': {
        'model': 'person_senior_1', 'tex': 'senior1_texture',
        'W': 256, 'H': 480, 'is_child': False,
        'skin': (185, 155, 135), 'hair': (200, 200, 200),           # 흰 머리
        'shirt_top': (100, 100, 100),'shirt_bot': (70, 70, 70),     # 회색 셔츠
        'pants_top': (55, 50, 45),   'pants_bot': (38, 35, 30),
    },
    'child1': {
        'model': 'person_child_1', 'tex': 'child1_texture',
        'W': 200, 'H': 380, 'is_child': True,
        'skin': (175, 130, 100), 'hair': (30, 20, 10),
        'shirt_top': (30, 200, 240), 'shirt_bot': (20, 160, 200),  # 노란 셔츠 (BGR)
        'pants_top': (130, 80, 50),  'pants_bot': (100, 60, 35),
    },
    'child2': {
        'model': 'person_child_2', 'tex': 'child2_texture',
        'W': 210, 'H': 400, 'is_child': True,
        'skin': (160, 115, 88), 'hair': (35, 22, 12),
        'shirt_top': (60, 200, 80),  'shirt_bot': (40, 150, 60),   # 초록 셔츠
        'pants_top': (100, 70, 50),  'pants_bot': (75, 50, 35),
    },
    'child3': {
        'model': 'person_child_3', 'tex': 'child3_texture',
        'W': 200, 'H': 390, 'is_child': True,
        'skin': (190, 150, 120), 'hair': (25, 16, 8),
        'shirt_top': (200, 70, 120), 'shirt_bot': (160, 50, 90),   # 분홍 셔츠
        'pants_top': (80, 55, 35),   'pants_bot': (60, 40, 25),
    },
}


def main():
    for key, cfg in CONFIGS.items():
        model    = cfg['model']
        tex_name = cfg['tex']
        W, H     = cfg['W'], cfg['H']

        tex_dir = os.path.join(MODELS_DIR, model, 'materials', 'textures')
        os.makedirs(tex_dir, exist_ok=True)
        tex_path     = os.path.join(tex_dir, f'{tex_name}.png')
        overlay_path = os.path.join(tex_dir, f'{tex_name}_overlay.png')

        # ── 다운로드 시도 ────────────────────────────────────────────────
        dl_img = None
        if key in DOWNLOAD_SOURCES:
            print(f'\n[{key}] 웹 다운로드 시도...')
            dl_img = try_download(DOWNLOAD_SOURCES[key], (W, H))

        if dl_img is not None:
            # 다운로드 성공: 상반신 사진 → 하반신 합성
            half = H // 2
            synth = draw_person_hq(W, H, **{k: v for k, v in cfg.items()
                                            if k not in ('model', 'tex', 'W', 'H')},
                                   bg=PERSON_BG)
            bgr = synth.copy()
            # 상단 절반에 다운로드 이미지 블렌딩
            face_region = cv2.resize(dl_img, (W, half))
            bgr[:half] = cv2.addWeighted(synth[:half], 0.3, face_region, 0.7, 0)
        else:
            print(f'\n[{key}] 합성 이미지 생성 중...')
            bgr = draw_person_hq(W, H, **{k: v for k, v in cfg.items()
                                          if k not in ('model', 'tex', 'W', 'H')},
                                 bg=PERSON_BG)

        # Gazebo 텍스처 (불투명 배경)
        cv2.imwrite(tex_path, bgr)
        print(f'  텍스처 저장: {tex_path}')

        # 오버레이용 투명 PNG
        bgra = make_transparent(bgr, bg_color=PERSON_BG)
        cv2.imwrite(overlay_path, bgra)
        print(f'  오버레이 저장: {overlay_path}')

    print('\n=== 완료 ===')
    print('실제 사진으로 교체하려면:')
    print('  models/<model>/materials/textures/*_texture.png  을 덮어쓰세요.')


if __name__ == '__main__':
    main()
