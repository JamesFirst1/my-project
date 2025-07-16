from ultralytics import YOLO
import cv2
import os
import time
import numpy  # 使用numpy创建测试图像

print("🔍 测试带GUI支持的OpenCV...")

# 测试OpenCV GUI功能
try:
    # 创建一个简单的窗口测试
    test_img = 255 * numpy.ones((300, 300, 3), dtype=numpy.uint8)  # 白色测试图像
    cv2.putText(test_img, "OpenCV GUI Test", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.imshow("GUI Test", test_img)
    cv2.waitKey(1000)  # 显示1秒
    cv2.destroyAllWindows()
    print("✅ OpenCV GUI功能正常!")
except Exception as e:
    print(f"⚠️ OpenCV GUI测试失败: {e}")
    print("将继续执行但可能无法显示窗口")

# 查找训练好的模型或使用预训练模型
model_paths = [
    "runs/detect/train/weights/best.pt",
    "runs/detect/train/weights/last.pt",
    "yolov8n.pt"  # 预训练模型(备用)
]

model_path = None
for path in model_paths:
    if os.path.exists(path):
        model_path = path
        break

if model_path:
    print(f"📦 使用模型: {model_path}")
    model = YOLO(model_path)
else:
    print("⚠️ 找不到任何模型，使用预训练模型...")
    model = YOLO("yolov8n.pt")

# 检查摄像头
print("📷 尝试打开摄像头...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 无法打开摄像头，尝试加载VOC图片...")
    
    # 从VOC数据集加载一些图片
    test_images = []
    voc_images_dir = "VOCdata/VOC2007/JPEGImages"
    if os.path.exists(voc_images_dir):
        for i, file_name in enumerate(sorted(os.listdir(voc_images_dir))):
            if file_name.lower().endswith(('.jpg', '.jpeg', '.png')) and i < 5:
                test_images.append(os.path.join(voc_images_dir, file_name))
    
    if test_images:
        print(f"🖼️ 加载了 {len(test_images)} 张测试图片")
        for i, img_path in enumerate(test_images):
            print(f"处理图片 {i+1}/{len(test_images)}: {os.path.basename(img_path)}")
            
            # 加载图片
            img = cv2.imread(img_path)
            if img is None:
                print(f"❌ 无法加载图片: {img_path}")
                continue
                
            # 检测
            start_time = time.time()
            results = model(img)
            elapsed = (time.time() - start_time) * 1000  # 毫秒
            
            # 绘制结果
            annotated_img = results[0].plot()
            
            # 保存结果
            os.makedirs("detection_results", exist_ok=True)
            save_path = f"detection_results/result_{i+1}.jpg"
            cv2.imwrite(save_path, annotated_img)
            print(f"✅ 结果已保存: {save_path} (处理时间: {elapsed:.1f}ms)")
            
            # 显示结果
            try:
                cv2.imshow(f"检测结果 {i+1}", annotated_img)
                key = cv2.waitKey(0)
                if key == 27 or key == ord('q'):  # ESC或q键退出
                    break
            except Exception as e:
                print(f"❌ 显示图像失败: {e}")
        
        cv2.destroyAllWindows()
    else:
        print("❌ 找不到任何测试图片")
else:
    print("🎥 摄像头已打开! 按'q'键退出")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 无法获取图像")
                break
            
            # 检测
            results = model(frame)
            
            # 绘制结果
            annotated_frame = results[0].plot()
            
            # 显示结果
            cv2.imshow("YOLOv8检测", annotated_frame)
            
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"❌ 摄像头处理错误: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

print("👋 程序已结束!")
