
import numpy as np
import cv2
from skimage.filters.rank import entropy
from skimage.morphology import disk

cap = cv2.VideoCapture('test_video.mp4')
i = 0

while(cap.isOpened()):
    ret, frame = cap.read()

    #frame = cv2.resize(frame,None,fx=0.25, fy=0.25, interpolation = cv2.INTER_AREA)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #ent_img = entropy(gray, disk(3))
    # softmax
    #ent_img = np.exp(ent_img - np.max(ent_img))
    #ent_img = ent_img / ent_img.sum()
    #ent_img = (ent_img - np.min(ent_img)) / (np.max(ent_img) - np.min(ent_img))
    #ent_img[ent_img < 0.9] = 0
    #ent_img[:ent_img.shape[0]*0.55,:] = 0
    
    cv2.imshow('frame',frame)
    cv2.imwrite('save/frame{}.jpg'.format(i), frame)
    
    i += 1
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()