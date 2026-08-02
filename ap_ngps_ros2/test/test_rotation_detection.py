#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from pathlib import Path


def create_test_images():
    img = np.zeros((400, 400, 3), dtype=np.uint8)
    
    cv.rectangle(img, (100, 100), (300, 300), (255, 255, 255), -1)
    
    cv.circle(img, (150, 150), 10, (0, 0, 255), -1)
    cv.circle(img, (250, 150), 10, (0, 255, 0), -1)
    cv.circle(img, (150, 250), 10, (255, 0, 0), -1)
    cv.circle(img, (250, 250), 10, (255, 255, 0), -1)
    
    rots = [0, 15, 30, 45, -15, -30, -45]
    rot_imgs = []
    
    for angle in rots:
        center = (img.shape[1] // 2, img.shape[0] // 2)
        rot_mat = cv.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv.warpAffine(img, rot_mat, (img.shape[1], img.shape[0]))
        rot_imgs.append((rotated, angle))
    
    return img, rot_imgs


def detect_rotation_homography(img1, img2):
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(gray1, None)
    kp2, des2 = sift.detectAndCompute(gray2, None)
    
    if des1 is None or des2 is None:
        return 0.0
    
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    
    good = []
    for mp in matches:
        if len(mp) == 2:
            m, n = mp
            if m.distance < 0.75 * n.distance:
                good.append(m)
    
    if len(good) < 4:
        return 0.0
    
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    
    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
    
    if M is None:
        return 0.0
    
    try:
        M_norm = M / M[2, 2]
        R = M_norm[:2, :2]
        
        sx = np.linalg.norm(R[:, 0])
        sy = np.linalg.norm(R[:, 1])
        
        if sx > 0 and sy > 0:
            R_norm = R / np.array([[sx], [sy]])
            theta_rad = np.arctan2(R_norm[1, 0], R_norm[0, 0])
            theta_deg = np.degrees(theta_rad)
        else:
            theta_deg = 0.0
            
    except Exception:
        theta_deg = 0.0
    
    return theta_deg


def detect_rotation_keypoints(img1, img2):
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(gray1, None)
    kp2, des2 = sift.detectAndCompute(gray2, None)
    
    if des1 is None or des2 is None:
        return 0.0
    
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    
    good = []
    for mp in matches:
        if len(mp) == 2:
            m, n = mp
            if m.distance < 0.75 * n.distance:
                good.append(m)
    
    if len(good) < 4:
        return 0.0
    
    rots = []
    for i in range(len(good)):
        for j in range(i + 1, len(good)):
            pt1_1 = np.array(kp1[good[i].queryIdx].pt)
            pt1_2 = np.array(kp1[good[j].queryIdx].pt)
            pt2_1 = np.array(kp2[good[i].trainIdx].pt)
            pt2_2 = np.array(kp2[good[j].trainIdx].pt)
            
            v1 = pt1_2 - pt1_1
            v2 = pt2_2 - pt2_1
            
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                cos_a = np.clip(cos_a, -1.0, 1.0)
                angle = np.arccos(cos_a)
                rots.append(np.degrees(angle))
    
    if len(rots) > 0:
        return np.median(rots)
    else:
        return 0.0


def detect_rotation_contour(img1, img2):
    gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(gray1, None)
    kp2, des2 = sift.detectAndCompute(gray2, None)
    
    if des1 is None or des2 is None:
        return 0.0
    
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    
    good = []
    for mp in matches:
        if len(mp) == 2:
            m, n = mp
            if m.distance < 0.75 * n.distance:
                good.append(m)
    
    if len(good) < 4:
        return 0.0
    
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    
    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
    
    if M is None:
        return 0.0
    
    h, w = img1.shape[:2]
    pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    dst = cv.perspectiveTransform(pts, M)
    
    try:
        rect = cv.minAreaRect(np.int32(dst))
        theta = rect[2]
        
        if theta < -45:
            theta = 90 + theta
        elif theta > 45:
            theta = -90 + theta
        
        return theta
        
    except Exception:
        return 0.0


def test_rotation_detection():
    print("Creating test images...")
    orig_img, rot_imgs = create_test_images()
    
    print("Testing rotation detection methods...")
    print("True Angle | Homography | Keypoints | Contour")
    print("-" * 50)
    
    for rot_img, true_angle in rot_imgs:
        hom_angle = detect_rotation_homography(orig_img, rot_img)
        kpt_angle = detect_rotation_keypoints(orig_img, rot_img)
        cnt_angle = detect_rotation_contour(orig_img, rot_img)
        
        print(f"{true_angle:8.1f}° | {hom_angle:8.1f}° | {kpt_angle:8.1f}° | {cnt_angle:8.1f}°")
    
    print("\nTest completed!")


if __name__ == "__main__":
    test_rotation_detection()