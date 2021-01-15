# -*- coding: utf-8 -*-
"""
@author: Clarence
"""
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.measurements import label
from sklearn.model_selection import train_test_split

heatmap_thresh = 16         # Threshold on density of bounding rects
bin_n, sect_n = (16, 16)     # Number of HOG bins


def prepare_data(test_size=0.25):
    src="/home/eaibot/share/hog_arr/aablue.jpg"
    blue = cv2.imread(src)
    src="/home/eaibot/share/hog_arr/aared.jpg"
    red = cv2.imread(src)
    src="/home/eaibot/share/hog_arr/aanull.jpg"
    null = cv2.imread(src)
  
    blueCells = np.hsplit(blue, blue.shape[1]/38)
    redCells = np.hsplit(red, red.shape[1]/38) 
    nullCells = np.hsplit(null, null.shape[1]/38) 
    
    trainCells = blueCells + redCells + nullCells
    hogdata = [list(map(hog,trainCells))]
    trainData = np.float32(hogdata).reshape(-1,bin_n*sect_n)
    responses = [1] * len(blueCells) + [2] * len(redCells)
    responses += [3] * len(nullCells)
    responses = np.array(responses)[:,np.newaxis]
    
    ds = train_test_split(trainData,responses,random_state=0,test_size=test_size)
    X, obs_Data, y, obs_y = ds
    svm = prepare_svm(X, y)    
    
    if test_size==0.0:    obs_Data, obs_y = (X, y)
    result = svm.predict(obs_Data)[1]
    result = np.array(result).flatten()
    result = result.astype(int).tolist()
    mask = (result==obs_y.flatten())
    correct = np.count_nonzero(mask)
    print('\nHOG Color Merit = %s%%\n' % round(correct*100.0/len(result)))

    return svm, trainData, responses

def hog_bin(mag, ang, ra=13, rb=25):
    bins = np.int32(bin_n*ang/(2*np.pi))    # quantizing binvalues in (0...16)
    bin_cells = bins[:ra,:ra], bins[ra:rb,:ra], bins[rb:,:ra]
    bin_cells += bins[:ra,ra:rb], bins[rb:,ra:rb]
    bin_cells += bins[:ra,rb:], bins[ra:rb,rb:], bins[rb:,rb:]
    mag_cells = mag[:ra,:ra], mag[ra:rb,:ra], mag[rb:,:ra]
    mag_cells += mag[:ra,ra:rb], mag[rb:,ra:rb]
    mag_cells += mag[:ra,rb:], mag[ra:rb,rb:], mag[rb:,rb:]
    
    return bin_cells, mag_cells

def hog(img):
    # signature: plt.plot(hog(hints.track[0]))
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
    mag_bgr, ang_bgr = cv2.cartToPolar(gx, gy)
    
    bin_cells0, mag_cells0 = hog_bin(mag_bgr[:,:,0], ang_bgr[:,:,0])
    bin_cells2, mag_cells2 = hog_bin(mag_bgr[:,:,2], ang_bgr[:,:,2])
    bin_cells = bin_cells0 + bin_cells2
    mag_cells = mag_cells0 + mag_cells2
    
    hist_cells = zip(bin_cells, mag_cells)
    hists = [np.bincount(b.ravel(), m.ravel(), bin_n) for b, m in hist_cells]
    hists = np.hstack(hists)     # hist is a 16*16 bytes vector
    norm = np.array(list(np.linalg.norm(c) for c in hists.reshape(-1,bin_n)))
    norm = norm[norm>np.median(norm)]
    norm = float(norm.mean()) if len(norm)>0 else 1.0
    return hists/norm

def prepare_svm(trainData, responses):
    svm = cv2.ml.SVM_create()
    svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setType(cv2.ml.SVM_C_SVC)
    svm.setC(2.67)
    svm.setGamma(5.383)

    svm.train(trainData, cv2.ml.ROW_SAMPLE, responses)
    svm.save('svm_data.dat')
    return svm

def hog_visualise(img, ra=13*2, rb=25*2):
    color = (192,192,192)
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
    imy, _ = cv2.cartToPolar(gx, gy)
    imy = imy.astype(np.uint8) 
    h, w = imy.shape[:2]
    cv2.line(imy, (0,ra), (w,ra), color, 1)
    cv2.line(imy, (0,rb-1), (w,rb-1), color, 1)
    return imy

def heat_maping(image, svm, code=(1,2), is_map=False, xy_window=(38,38)):
    # Calculate density of objects and draw heat map
    hot_windows = search_windows(image, svm, code, xy_window)

    imcopy = np.copy(image)
    for bbox in hot_windows:
        cv2.rectangle(imcopy, bbox[0], bbox[1], (0, 0, 255), 2)

    # find density of bounding rects
    heatmap = np.zeros(image.shape[:2], np.uint8)
    for box in hot_windows:
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1
   
    heatmap[heatmap <= heatmap_thresh] = 0

    if not is_map:
        # Draw final bounding rect
        labels = label(heatmap)
        draw_img = draw_labeled_rect(np.copy(image), labels)
        print('%d objects found' % (labels[1],))
        plt.imshow(labels[0], cmap='gray')
        plt.show()
        return draw_img
    else:
        return heatmap        

# Draw bounding rects based on labels
def draw_labeled_rect(img, labels):
    for car_number in range(1, labels[1]+1):
        # Identify x and y values of those pixels with each label value
        nonzero = (labels[0] == car_number).nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        bbox_w = (bbox[1][0] - bbox[0][0])
        bbox_h = (bbox[1][1] - bbox[0][1])

        # bounding rect acceptable aspect ratio range
        min_ar, max_ar = (0.7, 3.0)
        small_bbox_area, min_bbox_area  = (40*40, 10*10)

        # find aspect ratios, e.g. thin vertical box is likely not the object
        aspect_ratio = bbox_w / bbox_h # width / height
        bbox_area = bbox_w * bbox_h    # bounding area
        is_noise = (bbox_area < small_bbox_area)
        is_object = (bbox_area > min_bbox_area)    
        # Combine overall filters and draw rect of correct finding
        if aspect_ratio > min_ar and aspect_ratio < max_ar and not is_noise and is_object:
            pass
        cv2.rectangle(img, bbox[0], bbox[1], (0,180,192), 2)

    return img

def search_windows(img, svm, code, xy_window=(38, 38), scale=2):
    #1) Create an empty list to receive positive detection windows
    on_windows = []
    #2) Iterate over all windows in the list
    for row in slide_window(img, (xy_window[0]/scale,xy_window[1]/scale)):
        #3) Extract the test window from original image
        searchCells = []
        for rect in row:
            cell = cv2.resize(img[rect[0][1]:rect[1][1], rect[0][0]:rect[1][0]], xy_window)
            searchCells.append(cell)
        #4) Extract features for that window using single_img_features()
        hogdata = list(map(hog, searchCells))
        features = np.float32(hogdata).reshape(-1,bin_n*sect_n)
        #5) Predict using your classifier
        result = svm.predict(features)[1]
        result = np.array(result).flatten()
        result = result.astype(int).tolist()

        #6) If positive detections (result > 0) then save rect of window        
        for rect, guess in zip(row, result):
            if guess in code:    on_windows.append(rect)
            
    return on_windows

def slide_window(img, xy_window, xy_overlap=(3,3)):
    # set x, y to image size
    x_start_stop = (0, img.shape[1])
    y_start_stop = (0, img.shape[0])
    xspan = x_start_stop[1] - x_start_stop[0] - xy_window[0]
    yspan = y_start_stop[1] - y_start_stop[0] - xy_window[1]
    x0, y0 = (x_start_stop[0], y_start_stop[0])
    w, h = xy_overlap

    # Compute the number of windows in x/y
    window_list = []
    nx_windows, ny_windows = (np.int(xspan/w + 1), np.int(yspan/h + 1))

    for ys in range(ny_windows):
        starty = ys*h + y0
        endy = starty + xy_window[1]
        slide_rect = lambda xs: ((xs*w+x0,starty), (xs*w+x0+xy_window[0],endy))
        # Calculate window position
        slide = [slide_rect(xs) for xs in range(nx_windows)]
        window_list.append(slide)

    return window_list
