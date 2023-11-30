import os
from pathlib import Path
import numpy as np
import pytesseract
from pytesseract import Output
import cv2
import random



T = []
D = []
K = []
number_order = {}
result_list = False



def recognition(img):
    #boxes = pytesseract.image_to_data(img,config="--psm 6 -c tessedit_char_whitelist=TDK",lang='eng')
    boxes = pytesseract.image_to_data(img,config="--psm 10",lang='eng')
    for x,b in enumerate(boxes.splitlines()):
        if x!=0:
            b = b.split()
            #print(b)           conf level
            # if len(b) == 12 and int(float(b[10]))>=10 :
            if len(b) == 12:
                # x,y,w,h = int(b[6]),int(b[7]),int(b[8]),int(b[9])
                # cv2.rectangle(img,(x,y),(w+x,h+y),(0,0,255),3)
                # cv2.putText(img,b[11],(x,y),cv2.FONT_HERSHEY_COMPLEX,1,(50,50,255),2)
                text = b[11]
                #print(text)
                if "T" in text:
                    text = "T"
                elif "D" in text:
                    text = "D"
                elif "K" in text:
                    text = "K"
                else:
                     text = None
                return text

def take_photo(dir):
    vs = cv2.VideoCapture(1,cv2.CAP_DSHOW)
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



"""
start the main function
"""


def main():
    dirname, filename = os.path.split(os.path.abspath(__file__))
    pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"
    #print(dirname)

    take_photo(dirname)
    rec_contour=[]

    for number in range(0,6):
        frame = cv2.imread(rf"{dirname}\frame{number}.png")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        _, threshold = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

        img = threshold

        #filter and blurred
        kernel = np.ones((5,5),np.float32)/25
        blur_img = cv2.filter2D(frame.copy(),-1,kernel)
        canny_img = cv2.Canny(img, 20, 160)
        #cv2.imshow("canny",canny_img)
        contours, _ = cv2.findContours(
            canny_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            #if i==0:
                #	i=1
                #	continue
            approx = cv2.approxPolyDP(
                    contour, 0.01 * cv2.arcLength(contour, True), True)


            #find width/height = 1
            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = float(w)/h
            
            #find area
            area=cv2.contourArea(contour)
            extent=float(area)/(w*h)
            
            if len(approx) == 4 and aspect_ratio<1.2 and 0.9<aspect_ratio and extent>0.5:
                rec_contour.append(contour)
                # print(len(rec_contour))
                cv2.drawContours(img, [contour], 0, (0, 0, 255), 2)

        if rec_contour is not None:
            for contour in rec_contour:
                
                # using drawContours() function
                cv2.drawContours(img, [contour], 0, (0, 0, 255), 2)

                cv2.putText(img, 'Q', (x, y),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)
            
                # displaying the image after drawing contours
                # cv2.imshow('shapes', img)
                x,y,w,h = cv2.boundingRect(contour)
                
                # img after cutting
                pts1 = np.float32([[x,y],[x+w,y],[x,y+h],[x+w,y+h]])
                pts2 = np.float32([[0,0],[290,0],[0,290],[290,290]])
                M = cv2.getPerspectiveTransform(pts1,pts2)
                dst = cv2.warpPerspective(img.copy(),M,(290,290))
                cv2.imshow(f'result{contour}',dst)
                
                text = recognition(dst)
                #print(text,x,y)

                if text == "T" and len(T)<=6:
                    T.append(x)
                elif text == "D" and len(D)<=6:
                    D.append(x)
                elif text == "K" and len(K)<=6:
                    K.append(x)
            
                # print(f"T: {T}")
                # print(f"D: {D}")
                # print(f"K: {K}")

    if (len(T)>=5 or len(D)>=5 or len(K)>=5) and (len(T)*len(D)*len(K)!=0):
        T_avg = sum(T)/len(T)
        D_avg = sum(D)/len(D)
        K_avg = sum(K)/len(K)

        result = [["T",T_avg],["D",D_avg],["K",K_avg]]
        result.sort(key = lambda result: result[1])
        ans_list = []
        for i in range(len(result)):
            ans_list.append(result[i][0])

    else:
        ans_list = ["T","D","K"]
        random.shuffle(ans_list)
    
    print(ans_list)
    return ans_list

main()