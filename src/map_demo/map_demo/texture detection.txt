#!/usr/bin/env python3

import cv2
import numpy as np


CAMERA_PATH = "/dev/video0"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

# 只分析图像下半部分
ROI_START_RATIO = 0.5

# 待割区域像素总数低于该值时停车
STOP_THRESHOLD = 2000


def normalize_to_uint8(img):
    """
    将任意数值范围的图像归一化到 uint8 的 0～255。
    """
    img = img.astype(np.float32)

    min_val = np.min(img)
    max_val = np.max(img)

    if max_val - min_val < 1e-6:
        return np.zeros_like(img, dtype=np.uint8)

    output = (img - min_val) / (max_val - min_val) * 255.0

    return output.astype(np.uint8)


def get_external_grass_mask(frame):
    """
    提取外部草地区域。

    使用：
    1. HSV颜色范围
    2. Lab颜色空间中的绿色、黄绿色特征
    3. 两种掩膜融合
    """

    # ---------- HSV部分 ----------
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_green = np.array([25, 25, 20], dtype=np.uint8)
    upper_green = np.array([95, 255, 255], dtype=np.uint8)

    hsi_mask = cv2.inRange(
        hsv,
        lower_green,
        upper_green
    )

    # ---------- Lab部分 ----------
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    _, a_channel, b_channel = cv2.split(lab)

    # OpenCV Lab中：
    # a值较低表示偏绿
    # b值较高表示偏黄
    lab_green = (
        (a_channel < 135)
        & (b_channel > 115)
    ).astype(np.uint8) * 255

    # ---------- 掩膜融合 ----------
    fused = cv2.bitwise_or(
        hsi_mask,
        lab_green
    )

    # ---------- 形态学去噪 ----------
    kernel = np.ones(
        (5, 5),
        dtype=np.uint8
    )

    fused = cv2.morphologyEx(
        fused,
        cv2.MORPH_OPEN,
        kernel
    )

    fused = cv2.morphologyEx(
        fused,
        cv2.MORPH_CLOSE,
        kernel
    )

    return fused, hsi_mask, lab_green


def get_texture_score(frame):
    """
    计算纹理强度。

    使用：
    1. 局部灰度方差
    2. Sobel梯度
    3. 加权融合

    通常：
    高纹理区域更可能是未割草地；
    低纹理区域更可能是已割草地。
    """

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    # ---------- 局部方差 ----------
    gray_float = gray.astype(np.float32)

    local_mean = cv2.blur(
        gray_float,
        (15, 15)
    )

    local_mean_square = cv2.blur(
        gray_float * gray_float,
        (15, 15)
    )

    variance = (
        local_mean_square
        - local_mean * local_mean
    )

    # 防止浮点误差导致轻微负数
    variance = np.maximum(
        variance,
        0
    )

    variance_uint8 = normalize_to_uint8(
        variance
    )

    # ---------- 梯度强度 ----------
    gradient_x = cv2.Sobel(
        gray,
        cv2.CV_32F,
        1,
        0,
        ksize=3
    )

    gradient_y = cv2.Sobel(
        gray,
        cv2.CV_32F,
        0,
        1,
        ksize=3
    )

    gradient = cv2.magnitude(
        gradient_x,
        gradient_y
    )

    gradient_uint8 = normalize_to_uint8(
        gradient
    )

    # ---------- 纹理融合 ----------
    texture = cv2.addWeighted(
        variance_uint8,
        0.6,
        gradient_uint8,
        0.4,
        0
    )

    return (
        texture,
        variance_uint8,
        gradient_uint8
    )


def split_mowed_unmowed(frame, grass_mask):
    """
    在草地区域内部，根据纹理强度区分：

    高纹理：未割区域
    低纹理：已割区域
    """

    texture, _, _ = get_texture_score(
        frame
    )

    # 仅保留草地区域中的纹理
    texture_in_grass = cv2.bitwise_and(
        texture,
        texture,
        mask=grass_mask
    )

    grass_pixels = texture_in_grass[
        grass_mask > 0
    ]

    # 草地区域太少时，不进行分类
    if grass_pixels.size < 100:
        empty_mask = np.zeros_like(
            grass_mask
        )

        return (
            empty_mask.copy(),
            empty_mask.copy(),
            empty_mask.copy(),
            texture
        )

    # OTSU要求输入二维或一维uint8数组
    threshold_value, _ = cv2.threshold(
        grass_pixels,
        0,
        255,
        cv2.THRESH_BINARY
        + cv2.THRESH_OTSU
    )

    # 高纹理：未割区域
    unmowed = (
        (texture_in_grass > threshold_value)
        & (grass_mask > 0)
    ).astype(np.uint8) * 255

    # 低纹理：已割区域
    mowed = (
        (texture_in_grass <= threshold_value)
        & (grass_mask > 0)
    ).astype(np.uint8) * 255

    kernel = np.ones(
        (5, 5),
        dtype=np.uint8
    )

    # 未割区域去噪
    unmowed = cv2.morphologyEx(
        unmowed,
        cv2.MORPH_OPEN,
        kernel
    )

    unmowed = cv2.morphologyEx(
        unmowed,
        cv2.MORPH_CLOSE,
        kernel
    )

    # 已割区域去噪
    mowed = cv2.morphologyEx(
        mowed,
        cv2.MORPH_OPEN,
        kernel
    )

    mowed = cv2.morphologyEx(
        mowed,
        cv2.MORPH_CLOSE,
        kernel
    )

    # 未割区域边缘作为内部边界
    boundary = cv2.Canny(
        unmowed,
        50,
        150
    )

    return (
        mowed,
        unmowed,
        boundary,
        texture
    )


def decide_direction_from_unmowed(unmowed_mask):
    """
    根据图像下半部分未割区域的分布判断方向。

    中间最多：前进
    左边最多：左转
    右边最多：右转
    未割区域太少：停车
    """

    height, width = unmowed_mask.shape

    roi_top = int(
        height * ROI_START_RATIO
    )

    roi = unmowed_mask[
        roi_top:height,
        :
    ]

    roi_height, roi_width = roi.shape

    one_third = roi_width // 3
    two_thirds = 2 * roi_width // 3

    left_region = roi[
        :,
        0:one_third
    ]

    center_region = roi[
        :,
        one_third:two_thirds
    ]

    right_region = roi[
        :,
        two_thirds:roi_width
    ]

    left_score = cv2.countNonZero(
        left_region
    )

    center_score = cv2.countNonZero(
        center_region
    )

    right_score = cv2.countNonZero(
        right_region
    )

    total_score = (
        left_score
        + center_score
        + right_score
    )

    if total_score < STOP_THRESHOLD:
        direction = "STOP"

    elif (
        center_score >= left_score
        and center_score >= right_score
    ):
        direction = "FORWARD"

    elif left_score > right_score:
        direction = "LEFT"

    else:
        direction = "RIGHT"

    return (
        direction,
        left_score,
        center_score,
        right_score
    )


def draw_overlay(
    frame,
    grass_mask,
    mowed,
    unmowed,
    boundary,
    direction,
    scores
):
    """
    绘制最终结果。

    蓝色：已割区域
    绿色：未割区域
    红色：内部边界
    """

    del grass_mask  # 当前只用于接口一致性

    color_layer = frame.copy()

    # 已割区域：蓝色
    color_layer[mowed > 0] = (
        255,
        80,
        80
    )

    # 未割区域：绿色
    color_layer[unmowed > 0] = (
        80,
        255,
        80
    )

    # 边界：红色
    color_layer[boundary > 0] = (
        0,
        0,
        255
    )

    result = cv2.addWeighted(
        frame,
        0.55,
        color_layer,
        0.45,
        0
    )

    height, width = result.shape[:2]

    roi_top = int(
        height * ROI_START_RATIO
    )

    one_third = width // 3
    two_thirds = 2 * width // 3

    # 绘制下半部分三分区
    cv2.line(
        result,
        (one_third, roi_top),
        (one_third, height),
        (255, 255, 255),
        2
    )

    cv2.line(
        result,
        (two_thirds, roi_top),
        (two_thirds, height),
        (255, 255, 255),
        2
    )

    cv2.line(
        result,
        (0, roi_top),
        (width, roi_top),
        (255, 255, 255),
        2
    )

    left_score, center_score, right_score = scores

    cv2.putText(
        result,
        f"CMD: {direction}",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 0, 255),
        3
    )

    cv2.putText(
        result,
        (
            f"Unmowed L:{left_score} "
            f"C:{center_score} "
            f"R:{right_score}"
        ),
        (30, 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        result,
        "Blue=Mowed Green=Unmowed Red=Boundary",
        (30, 135),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2
    )

    return result


def create_camera():
    """
    使用GStreamer打开摄像头。

    摄像头输出MJPG；
    GStreamer完成JPEG解码并转换为BGR；
    OpenCV直接接收BGR图像。
    """

    pipeline = (
        f"v4l2src device={CAMERA_PATH} io-mode=2 ! "
        f"image/jpeg,"
        f"width={FRAME_WIDTH},"
        f"height={FRAME_HEIGHT},"
        f"framerate={FPS}/1 ! "
        "jpegparse ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink "
        "drop=true "
        "max-buffers=1 "
        "sync=false"
    )

    print("GStreamer pipeline:")
    print(pipeline)

    camera = cv2.VideoCapture(
        pipeline,
        cv2.CAP_GSTREAMER
    )

    if not camera.isOpened():
        camera.release()
        return None

    return camera


def main(args=None):
    """
    程序入口。
    """

    del args

    camera = create_camera()

    if camera is None:
        print(
            f"camera open failed: {CAMERA_PATH}"
        )
        print(
            "请确认没有其他程序占用摄像头。"
        )
        return

    print("Fusion texture vision started")
    print(
        f"Backend: {camera.getBackendName()}"
    )
    print(
        f"Camera: {CAMERA_PATH}"
    )
    print(
        f"Resolution: "
        f"{FRAME_WIDTH}x{FRAME_HEIGHT}"
    )
    print(
        f"FPS: {FPS}"
    )
    print("Press ESC to quit")

    try:
        while True:
            ret, frame = camera.read()

            if not ret or frame is None:
                print("camera read failed")
                break

            # 正常情况下GStreamer已经输出指定尺寸
            if (
                frame.shape[1] != FRAME_WIDTH
                or frame.shape[0] != FRAME_HEIGHT
            ):
                frame = cv2.resize(
                    frame,
                    (
                        FRAME_WIDTH,
                        FRAME_HEIGHT
                    )
                )

            (
                grass_mask,
                hsi_mask,
                lab_mask
            ) = get_external_grass_mask(
                frame
            )

            (
                mowed,
                unmowed,
                boundary,
                texture
            ) = split_mowed_unmowed(
                frame,
                grass_mask
            )

            (
                direction,
                left_score,
                center_score,
                right_score
            ) = decide_direction_from_unmowed(
                unmowed
            )

            overlay = draw_overlay(
                frame=frame,
                grass_mask=grass_mask,
                mowed=mowed,
                unmowed=unmowed,
                boundary=boundary,
                direction=direction,
                scores=(
                    left_score,
                    center_score,
                    right_score
                )
            )

            cv2.imshow(
                "01 original",
                frame
            )

            cv2.imshow(
                "02 HSI mask",
                hsi_mask
            )

            cv2.imshow(
                "03 Lab mask",
                lab_mask
            )

            cv2.imshow(
                "04 external grass mask fusion",
                grass_mask
            )

            cv2.imshow(
                "05 texture score",
                texture
            )

            cv2.imshow(
                "06 mowed area",
                mowed
            )

            cv2.imshow(
                "07 unmowed area",
                unmowed
            )

            cv2.imshow(
                "08 final overlay",
                overlay
            )

            print(
                f"CMD={direction}, "
                f"unmowed L={left_score}, "
                f"C={center_score}, "
                f"R={right_score}"
            )

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                print("ESC pressed")
                break

    except KeyboardInterrupt:
        print("\nProgram interrupted")

    finally:
        camera.release()
        cv2.destroyAllWindows()
        print("Camera released")


if __name__ == "__main__":
    main()