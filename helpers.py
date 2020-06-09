import cv2 as cv


def binarize(img, d=0, hls_start=(0, 0, 0), hls_end=(255, 255, 255), gray_start=0, gray_end=255):
    hls = cv.cvtColor(img, cv.COLOR_BGR2HLS)
    binaryh = cv.inRange(hls, hls_start, hls_end)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    binaryg = cv.inRange(gray, gray_start, gray_end)

    binary = cv.bitwise_and(binaryg, binaryh)

    if d:
        cv.imshow('hls', binaryh)
        cv.imshow('gray', binaryg)
        cv.imshow('bin', binary)
    return binary


def trans_perspective(binary, trap, rect, size, d=0):
    M = cv.getPerspectiveTransform(trap, rect)
    perspective = cv.warpPerspective(binary, M, size, flags=cv.INTER_LINEAR)
    if d:
        cv.imshow('perspective', perspective)
    return perspective


def constrain(val, minv, maxv):
    return min(maxv, max(minv, val))