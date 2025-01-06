import cv2
import numpy
import numpy as np
 
 
def nothing(*arg):
    pass
 
 
icol = (18, 0, 196, 36, 255, 255)
 
#path = "test/cruise/"
# Show the original image.
#frame = path+str(961)+'.jpg'
#frame = cv2.imread(frame)
l = [16, 45, 65]  # [17, 55, 128]#阈值
h = [44, 255, 255]  # [24, 255, 255]#阈值
 
def zh_ch(string):
    return string.encode("gbk").decode('UTF-8', errors='ignore')
 
# mm = 320
cap = cv2.VideoCapture(0)
def XunX(img,SX):
    cv2.imshow('frame', img)
    # Blur methods available, comment or uncomment to try different blur methods.
    frameBGR = cv2.GaussianBlur(img, (7, 7), 0)
 
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # HSV values to define a colour range.
 
    colorLow = numpy.array([16, 45, 65] )  # [17, 55, 128]
    colorHigh = numpy.array([44, 225, 225])  # [24, 225, 225]
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # Show the first mask
    cv2.imshow('mask-plain', mask)
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    cv2.imshow('mask-plain', mask)
    # Find_Line(mask)
 
    left = np.array([])
    right = np.array([])
    leftb = np.array([])
    rightb = np.array([])
    medim = np.array([])
    # len = []#记录边线丢失
    le = [[]]
    ri = [[]]
    zz = 0  # 记录左缺陷时上一个坐标
    yy = 639
    SX = SX  # 开始扫线时的y坐标
    SXp = np.array([])
    SS = 0
    left_up = left_down = right_up = right_down = 0
    for i in range(480, 1, -1):
        # 扫左线
        for j in range(SX + 1, 0, -1):
            # 中线：319
            if (mask[i - 1][j] == 0) and (mask[i - 1][j - 1] == 255):
                # 记录左跳变点的列值
                left = np.append(left, j)
                leftb = np.append(leftb, j)
                zz = j
                break
            # 现在缺线
            elif (j == 1):
                lk = 0
                left = np.append(left, lk)
                leftb = np.append(leftb, lk)
 
        # 扫右线
        for j1 in range(SX + 2, 639, 1):
            if (mask[i - 1][j1] == 0) and (mask[i - 1][j1 + 1] == 255):
                # 记录右跳变点的列值
                right = np.append(right, j1)
                rightb = np.append(rightb, j1)
                yy = j1
 
                break
            # 缺线
            elif (j1 == 638):
                lk = 639
                rightb = np.append(rightb, lk)
                right = np.append(right, lk)
        if (left[480 - i] == 0 and right[480 - i] != 639 and mask[i - 2][0] != 0 ):
                    break
        # if (right[480 - i] == 639 and left[480 - i] != 0 and mask[i - 2][639] != 255 ):
        #             break
        SX = int((left[480 - i] + right[480 - i]) / 2)
        SXp = np.append(SXp, SX)
    # 找拐点
    sm = 30
    for i in range(len(left) - 10):
        if (left[i] == 0 and left[i + 3] == 0 and left[i + 5] > 0 and left[i + 10] > 0):
            left_up = 1
            left_up1 = (i + min(len(left[i+2:]),sm), left[i + min(len(left[i+2:]),sm)])  # 480 - i-sm
        if (left[i] > 0 and left[i + 3] > 0 and left[i + 5] == 0 and left[i + 10] == 0):
            left_down = 1
            left_down1 = (i - min(i,sm), left[i - min(i,sm)])  # 480 - i+sm
        if (right[i] == 639 and right[i + 3] == 639 and right[i + 5] < 639 and right[i + 10] <= 639):
            right_up = 1
            right_up1 = (i + min(len(left[i+2:]),sm), right[i + min(len(left[i+2:]),sm)])
        if (right[i] < 639 and right[i + 3] < 639 and right[i + 5] == 639 and right[i + 10] == 639):
            right_down = 1
            right_down1 = (i - min(i,sm), right[i - min(i,sm)])  # -1
    # 判断元素：补线操作
    print(left_up, left_down, right_up, right_down)
    if (left_up and not left_down):
        left_down1 = (0, left_up1[1])
    elif (not left_up and left_down):
        left_up1 = (479, left_down1[1])
    if (right_up and not right_down):
        right_down1 = (0, right_up1[1])
    elif (not right_up and right_down):
        right_up1 = (479, right_down1[1])
    # 左右的上下拐点同时出现
    #if (left_up and right_up) or (left_down and right_down):
    if (left_up or right_up) or (left_down or right_down):
        # if(right_up1[0] - left_up1[0] <= 20):
        for k in range(len(leftb)):
            if (k >= left_down1[0] and k <= left_up1[0]) or (k <= left_down1[0] and k >= left_up1[0]):
                leftb[k] = ((left_up1[1] - left_down1[1]) / (left_up1[0] -
                                                             left_down1[0])) * (k - left_up1[0]) + left_up1[1]
            if (k >= right_down1[0] and k <= right_up1[0]) or (k <= right_down1[0] and k >= right_up1[0]):
                rightb[k] = ((right_up1[1] - right_down1[1]) / (right_up1[0] -
                                                                right_down1[0])) * (k - right_up1[0]) + right_up1[1]
    # # 同时出现左上下，或右上下：
    # if (left_up and left_down) or (right_up and right_down):
    #     # if(right_up1[0] - left_up1[0] <= 20):
    #     for k in range(len(leftb)):
    #         if (k >= left_down1[0] and k <= left_up1[0]) or (k <= left_down1[0] and k >= left_up1[0]):
    #             leftb[k] = ((left_up1[1] - left_down1[1]) / (left_up1[0] -
    #                                                          left_down1[0])) * (k - left_up1[0]) + left_up1[1]
    #         if (k >= right_down1[0] and k <= right_up1[0]) or (k <= right_down1[0] and k >= right_up1[0]):
    #             rightb[k] = ((right_up1[1] - right_down1[1]) / (right_up1[0] -
    #                                                             right_down1[0])) * (k - right_up1[0]) + right_up1[1]
    # left_up = left_down = right_up = right_down = 0
    #  if
 
    medim = (leftb + rightb) / 2
    #  img = [mask,mask3,mask4,mask5]
    pl = ['left', 'right', 'l&r', 'no']
    o = 0
    img = mask
    cg = len(left)-1
    print(cg,len(leftb),len(right))
    for k in range(cg, -1, -1):
        point = (int(medim[k]), 479 - k)  # 左
        # point2 = (SXp[k], 478-k)
        point3 = (int(leftb[k]), 479 - k)  # 中
        point1 = (int(rightb[k]), 479 - k)  # 右
        cv2.circle(img, point, 1, (255, 0, 255), 0)
        cv2.circle(img, point1, 1, (255, 0, 255), 0)
        cv2.circle(img, point3, 1, (255, 0, 255), 0)
    cv2.imshow(pl[o], img)
    cv2.waitKey(100000)
    return SX,(leftb + rightb) / 2,SXp
 
mm = 320#扫线开始的坐标
cap = cv2.VideoCapture(0)
while True:
    test, frame = cap.read()
 
    print(frame.shape)
    mm,XB,SXp = XunX(frame, mm)
cv2.destroyAllWindows()
 
 