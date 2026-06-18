import cv2
import numpy as np
import sys

def process_map(input_file, output_file):
    # 맵 로드
    img = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("Error: 파일을 찾을 수 없습니다.")
        return

    # [꼼수 1] 길 넓히기 (팽창(Dilate) 적용: 검은색 벽을 얇게, 흰색 길을 넓게)
    # kernel 크기를 키우면 통로가 더 넓어집니다.
    kernel = np.ones((2, 2), np.uint8)
    processed_img = cv2.dilate(img, kernel, iterations=1)

    # [꼼수 2] 노이즈 제거 (작은 장애물들 무시)
    processed_img = cv2.morphologyEx(processed_img, cv2.MORPH_OPEN, kernel)

    # 결과 저장
    cv2.imwrite(output_file, processed_img)
    print(f"완료! {output_file} 파일이 생성되었습니다.")

if __name__ == "__main__":
    # 실행 예시: python3 fix_map.py mart_map.pgm mart_map_fixed.pgm
    process_map(sys.argv[1], sys.argv[2])