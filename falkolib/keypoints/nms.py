'''
Created on Apr 26, 2024

@author: seb
'''
import math

def NMSKeypoint(scores, ranges, angle_inc, ibeg, iend, radius, minvalue):
    candidates = []
        
    i = ibeg
    imax = ibeg
    while(i < iend):
        if 147 == i:
            print('a ver')
        
        win = 0
        if radius >= ranges[i]:
            win = math.floor(math.asin(0.8) / angle_inc)
        else:
            win = math.floor(math.asin(radius / ranges[i]) / angle_inc)

        jmax = i

        if imax + win < i:
            jbeg = i - win if i >= win else 0
            imax = i
        else:
            jbeg = i

        if (i + win + 1 < iend):
            jend = i + win + 1
        else:
            jend = iend
        
        for j in range(jbeg, jend):
            if (scores[j] > scores[jmax]):
                jmax = j
        
        if (scores[jmax] >= scores[imax]):
            imax = jmax
        else:
            imax = imax

        if (i == imax) and (scores[i] > minvalue):
            candidates.append(i)
        if (jmax > i):
            i = jmax;
        else:
            i += 1
    
    i1 = 0
    i2 = 0
    counter = 0

    peaks = candidates
    # peaks = []
    # for i1 in range(0, len(candidates)):
    #     if (scores[candidates[i2]] == scores[candidates[i1]]):
    #         counter += 1
    #         if (2 * abs(i2 - i1) > counter):
    #             i2 += 1
    #     else:
    #         peaks.append(candidates[i2])
    #         i2 = i1
    #         counter = 0
    #
    # if (i2 != len(candidates)):
    #     peaks.append(candidates[i2])
    
    return peaks
        
