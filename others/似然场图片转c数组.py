from PIL import Image
import numpy as np

# 打开图像并转换为灰度模式
image_path = 'LikelihoodField.png'
img = Image.open(image_path).convert('L')
width, height = img.size

# 将图像转换为NumPy数组以便处理
img_array = np.array(img)

# 开始生成C语言数组文件
with open('image_array.c', 'w') as file:
    file.write('unsigned char image_array[{}][{}] = {{\n'.format(height, width))
    for row in img_array:
        row_string = '{' + ', '.join(str(val) for val in row) + '},\n'
        file.write(row_string)
    file.write('};')

print("C语言数组文件已生成。")
