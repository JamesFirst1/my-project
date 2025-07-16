from ultralytics import YOLO
import cv2
import os
import time

# VOC类别中文名称
VOC_CLASSES_CN = [
    "飞机", "自行车", "鸟", "船", "瓶子", "公交车", "汽车", "猫", 
    "椅子", "牛", "餐桌", "狗", "马", "摩托车", "人",
    "盆栽", "羊", "沙发", "火车", "电视"
]

print("📦 加载YOLOv8模型...")

# 查找训练模型
model_paths = [
    "runs/detect/train/weights/best.pt",  # 标准位置
    "runs/detect/train/weights/last.pt",  # 最后一次保存的模型
    "yolov8n.pt"                          # 预训练模型(备用)
]

model_path = None
for path in model_paths:
    if os.path.exists(path):
        model_path = path
        break

if model_path:
    print(f"✅ 使用模型: {model_path}")
    model = YOLO(model_path)
else:
    print("⚠️ 找不到训练好的模型，使用预训练模型...")
    model = YOLO("yolov8n.pt")

# 检查是否可以打开摄像头
print("📷 尝试打开摄像头...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 无法打开摄像头！尝试加载示例图片...")
    
    # 从VOC数据集加载一些图片进行测试
    test_images = []
    voc_images_dir = "VOCdata/VOC2007/JPEGImages"
    if os.path.exists(voc_images_dir):
        for i, file_name in enumerate(sorted(os.listdir(voc_images_dir))):
            if file_name.lower().endswith(('.jpg', '.jpeg', '.png')) and i < 5:  # 只加载5张图片
                test_images.append(os.path.join(voc_images_dir, file_name))
    
    if test_images:
        print(f"🖼️ 加载了 {len(test_images)} 张测试图片")
        for i, img_path in enumerate(test_images):
            print(f"处理图片 {i+1}/{len(test_images)}: {img_path}")
            img = cv2.imread(img_path)
            
            # 检测
            start_time = time.time()
            results = model(img)
            elapsed = (time.time() - start_time) * 1000  # 毫秒
            
            # 绘制结果
            annotated_img = results[0].plot()
            
            # 保存结果
            save_path = f"detection_result_{i+1}.jpg"
            cv2.imwrite(save_path, annotated_img)
            print(f"✅ 结果已保存: {save_path} (处理时间: {elapsed:.1f}ms)")
            
            # 尝试显示结果
            try:
                cv2.imshow(f"检测结果 {i+1}", annotated_img)
                cv2.waitKey(2000)  # 显示2秒
            except:
                print("⚠️ 无法显示图像，仅保存结果")
        
        cv2.destroyAllWindows()
    else:
        print("❌ 找不到任何示例图片")
    exit()

print("🎉 摄像头已准备就绪! 按'q'退出")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 无法获取图像!")
        break
    
    # 检测对象
    results = model(frame)
    
    # 绘制结果
    annotated_frame = results[0].plot()
    
    # 显示结果
    cv2.imshow("YOLOv8 VOC检测", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("👋 程序已结束!")
