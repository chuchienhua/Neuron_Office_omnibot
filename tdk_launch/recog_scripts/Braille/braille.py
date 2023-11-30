import  cv2
import  numpy as np
from matplotlib import pyplot as plt
from collections import OrderedDict
import os

def take_photo(dir):
    # vs = cv2.VideoCapture(4,cv2.CAP_DSHOW)
    vs = cv2.VideoCapture(0)
    count = 0

    while(count<6):
        ret, frame = vs.read()
        cv2.imshow('frame', frame)
        cv2.imwrite(f'{dir}/frame{count}.png', frame)
        count = count + 1
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
                # After the loop release the cap object
    vs.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def braille():
    dirname, filename = os.path.split(os.path.abspath(__file__))
    take_photo(dirname)
    img= cv2.imread(f"{dirname}/frame0.png", cv2.IMREAD_GRAYSCALE)
    ret, th1 = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

    template = cv2.imread(f"{dirname}/6_template.jpg",0)
    w, h = template.shape[::-1]
    d = {'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0}

    '''
    res = cv2.matchTemplate(img, template, eval('cv2.TM_CCOEFF_NORMED'))
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(img, top_left, bottom_right, 255, 2)
    #plt.subplot(121), plt.imshow(res, cmap='gray')
    #plt.subplot(122), plt.imshow(img, cmap='gray')
    #plt.imshow(img, cmap='gray')
    #plt.show()

    '''

    # 逐步比對template 1 - 6
    for i in range(1,7):
        temp_num = str(i) + "_template.jpg"
        #print(temp_num)
        #print(f"{dirname}/{temp_num}")
        template = cv2.imread(f"{dirname}/{temp_num}", 0)
        res = cv2.matchTemplate(img, template, eval('cv2.TM_CCOEFF_NORMED'))
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        d[str(i)] = top_left


    #用x值 去做排序
    ans = (sorted(d,key=lambda x:d[x][0]))
    return ans
    





    '''
    params = cv2.SimpleBlobDetector_Params()
    params.filterByCircularity = True
    params.minCircularity = 0.5

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(th1)
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    print(keypoints)
    '''

    '''
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
                'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

    for meth in methods:
        img2 = th1.copy()
        method = eval(meth)
        print(method)

        # Apply template Matching
        res = cv2.matchTemplate(img, template, method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(img, top_left, bottom_right, 255, 2)
        plt.subplot(121), plt.imshow(res, cmap='gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122), plt.imshow(img, cmap='gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle(meth)
        plt.show()
        
        
        '''



    '''
    plt.imshow(th1, 'gray')
    plt.show()
    '''
print(braille())