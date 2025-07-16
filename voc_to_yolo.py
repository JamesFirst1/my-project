import os
import xml.etree.ElementTree as ET
from tqdm import tqdm
import shutil

# VOC类别
VOC_CLASSES = [
    "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", 
    "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person",
    "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

def convert_box(size, box):
    dw = 1.0 / size[0]
    dh = 1.0 / size[1]
    x = (box[0] + box[2]) / 2.0
    y = (box[1] + box[3]) / 2.0
    w = box[2] - box[0]
    h = box[3] - box[1]
    x = x * dw
    w = w * dw
    y = y * dh
    h = h * dh
    return (x, y, w, h)

def convert_voc_annotation(data_path, output_path, image_set):
    os.makedirs(os.path.join(output_path, 'labels', image_set), exist_ok=True)
    os.makedirs(os.path.join(output_path, 'images', image_set), exist_ok=True)
    
    with open(os.path.join(data_path, 'VOC2007', 'ImageSets', 'Main', f'{image_set}.txt')) as f:
        image_ids = f.read().strip().split()
    
    for image_id in tqdm(image_ids, desc=f"Converting {image_set}"):
        # 读取XML文件
        annotation_file = os.path.join(data_path, 'VOC2007', 'Annotations', f'{image_id}.xml')
        
        # 复制图像文件
        image_file = os.path.join(data_path, 'VOC2007', 'JPEGImages', f'{image_id}.jpg')
        shutil.copy(image_file, os.path.join(output_path, 'images', image_set, f'{image_id}.jpg'))
        
        tree = ET.parse(annotation_file)
        root = tree.getroot()
        
        size = root.find('size')
        w = int(size.find('width').text)
        h = int(size.find('height').text)
        
        # 创建YOLO格式的标签文件
        out_file = open(os.path.join(output_path, 'labels', image_set, f'{image_id}.txt'), 'w')
        
        for obj in root.iter('object'):
            cls = obj.find('name').text
            if cls not in VOC_CLASSES:
                continue
            cls_id = VOC_CLASSES.index(cls)
            xmlbox = obj.find('bndbox')
            b = (float(xmlbox.find('xmin').text), float(xmlbox.find('ymin').text),
                 float(xmlbox.find('xmax').text), float(xmlbox.find('ymax').text))
            bb = convert_box((w, h), b)
            out_file.write(f"{cls_id} {' '.join([str(round(a, 6)) for a in bb])}\n")
        
        out_file.close()

# 主函数
def main():
    # 路径设置
    voc_path = "VOCdata"  # VOCdata目录的位置
    output_path = "VOCyolo"  # 输出目录
    
    # 转换训练集、验证集和测试集
    convert_voc_annotation(voc_path, output_path, "train")
    convert_voc_annotation(voc_path, output_path, "val")
    
    # 如果有test.txt文件，也转换测试集
    test_file = os.path.join(voc_path, 'VOC2007', 'ImageSets', 'Main', 'test.txt')
    if os.path.exists(test_file):
        convert_voc_annotation(voc_path, output_path, "test")
    
    # 创建YAML配置文件
    with open(os.path.join(output_path, "voc.yaml"), 'w') as yaml_file:
        yaml_content = f"""# VOC数据集配置
path: {os.path.abspath(output_path)}
train: images/train
val: images/val

# 类别
nc: {len(VOC_CLASSES)}
names: {VOC_CLASSES}
"""
        yaml_file.write(yaml_content)
    
    print(f"转换完成！配置文件保存在 {os.path.join(output_path, 'voc.yaml')}")

if __name__ == "__main__":
    main()
