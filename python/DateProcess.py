import numpy as np
import math
import scipy.signal as ss

class DateProcess(object):
    def __init__(self, data):
        self.data = data

    def findpeaks(self, lowbound=-99999, upperbound=99999, pro=0.05,dis = 8):
        data = self.data
        alllist = ss.find_peaks(data, prominence=pro,distance=dis)[0]
        index = []
        for each in alllist:
            if data[each] >= lowbound and data[each] <= upperbound:
                index.append(each)

        points = [data[i] for i in index]
        return index, points

    def findvalleies(self, lowbound=-99999, upperbound=99999, pro=0.05,dis = 8):
        data = [-l for l in self.data]
        alllist = ss.find_peaks(data, prominence=pro,distance=dis)[0]
        index = []
        for each in alllist:
            if data[each] <= -lowbound and data[each] >= -upperbound:
                index.append(each)

        points = [self.data[i] for i in index]
        return index, points

    def findzeropoints(self, rf=None):
        data = self.data
        points,points_index = [],[]
        for i in range(1, len(data)):
            if (data[i-1] >= 0 and data[i] < 0) and (rf is 'fall' or rf is None) or\
                (data[i-1] <= 0 and data[i] > 0) and (rf is 'rise' or rf is None):
                points_index.append(i-1)
                # points.append(data[i])
        pi_f,p_f = self.findnearzeropoints()
        points_index = list(set(points_index).difference(set(pi_f)))

        points = [data[i] for i in points_index]
        return points_index, points
    
    def findnearzeropoints(self,minnum=10,upperbound=0.05,lowbound=-0.05):
        data = self.data

        count = False
        buffer,buffer2 = [],[]
        for i in range(0, len(data)):
            if data[i]<=upperbound and data[i]>=lowbound:
                buffer2.append(i)

        points_index = []
        for i in range(1,len(buffer2)):
            if len(buffer)==minnum:
                count = True
            if buffer2[i]- buffer2[i-1] != 1:
                if count:
                    points_index.extend(buffer)
                    count = False
                buffer.clear()
            buffer.append(buffer2[i])

        points = [data[i] for i in points_index]
        return points_index, points
