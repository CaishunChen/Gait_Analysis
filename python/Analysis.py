from DateProcess import*
from Quaternion import*
import numpy,copy
class Pitch_period(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.start_valley = -1
        self.zero_rise = -1
        self.peak = -1
        self.nearzero = []
        self.tiny_peak = -1
        self.zero_fall = -1
        self.end_valley = -1
    # def __str__(self):
    #     return "[start:%d\nzero_rise:%d\npeak:%d\nnearzero:%d\n"% \
    #     (self.start_valley,self.zero_rise,self.peak)
    def tolist(self):
        res = [self.start_valley,self.zero_rise,self.peak]
        res.extend(self.nearzero)
        res.append(self.tiny_peak)
        res.append(self.zero_fall)
        res.append(self.end_valley)
        for each in res:
            if each == -1:
                res.remove(-1)
        return res
        
class Analysis(object):
    def __init__(self,ax, ay, az, gx, gy, gz,frq):
        self.ax,self.ay,self.az = ax,ay,az
        self.gx,self.gy,self.gz = gx,gy,gz
        self.num = min(len(ax),len(ay),len(az),len(gx),len(gy),len(gz))
        self.frq = frq

        self.getPRY()
        self.getAxyz()
        self.getVS()

        self.somecaculate()
        self.marktimeline()
        self.seperatetimeline()

    # 四元数姿态识别(YAW轴不准确)
    def getPRY(self):
        self.q = Quaternion(100, 0.002, 1/self.frq)
        self.pitch, self.yaw, self.roll = [], [], []
        self.q4 = []
        for index in range(self.num):
            r, p, y, qur = self.q.getAngle(self.ax[index], self.ay[index], self.az[index], self.gx[index], self.gy[index], self.gz[index])
            self.roll.append(r)    
            self.pitch.append(p)
            self.yaw.append(y)
            self.q4.append(qur)

    # 将相对坐标系上的加速度转换为绝对坐标系上的加速度
    def getAxyz(self):
        self.Ax,self.Ay,self.Az = [],[],[]
        for i in range(len(self.q4)):
            tmp = self.q.transform(self.q4[i],self.ax[i],self.ay[i],self.az[i])
            # tmp =list(tmp)
            # print(tmp[0])
            self.Ax.append(tmp[0])
            self.Ay.append(tmp[1])
            self.Az.append(tmp[2])
            
    # 对加速度速度积分，得到速度和位移
    def getVS(self):
        Vx,Vy,Vz = [0], [0], [0]
        Sx,Sy,Sz = [0], [0], [0]

        for i in range(1,self.num):
            temp_x = Vx[i-1]+(self.Ax[i]+self.Ax[i-1])/2/self.frq
            temp_y = Vy[i-1]+(self.Ay[i]+self.Ay[i-1])/2/self.frq
            temp_z = Vz[i-1]+(self.Az[i]+self.Az[i-1])/2/self.frq
            Vx.append(temp_x)
            Vy.append(temp_y)
            Vz.append(temp_z)
            if Vx[i] < 0:
                Vx[i] = 0

        for i in range(1,self.num):
            temp_x = Sx[i-1]+(Vx[i]+Vx[i-1])/2/self.frq
            temp_y = Sy[i-1]+(Vy[i]+Vy[i-1])/2/self.frq
            temp_z = Sz[i-1]+(Vz[i]+Vz[i-1])/2/self.frq
            Sx.append(temp_x)
            Sy.append(temp_y)
            Sz.append(temp_z)   

        self.Vx, self.Vy,self.Vz = Vx.copy(), Vy.copy(),Vz.copy()
        self.Sx, self.Sy,self.Sz =Sx.copy(), Sy.copy(),Sz.copy()


    #处理pitch轴数据，寻找关键点
    def somecaculate(self):
        dp = DateProcess(self.pitch)
        self.pitch_valley_index,self.pitch_valley = dp.findvalleies(upperbound=-0.4)
        self.pitch_peak_index,self.pitch_peak = dp.findpeaks(lowbound=0.45,dis=20)
        self.pitch_tinypeak_index,self.pitch_tinypeak = dp.findpeaks(lowbound=0.05,upperbound=0.45,dis=20)
        self.pitch_nearzeros_index,self.pitch_nearzeros = dp.findnearzeropoints(minnum = 10,upperbound=0.2,lowbound=-0.1)
        self.pitch_risezero_index,self.pitch_risezero = dp.findzeropoints('rise')
        self.pitch_fallzero_index,self.pitch_fallzero = dp.findzeropoints('fall')

        print(len(self.pitch_peak_index))
        print(len(self.pitch_valley_index))
        print(len(self.pitch_nearzeros_index))
    
    def marktimeline(self):
        timeline = ['' for i in range(len(self.pitch))]
        for each in self.pitch_nearzeros_index:
            timeline[each] = 'NEAR_ZERO'
        for each in self.pitch_valley_index:
            timeline[each] = 'VALLEY'
        for each in self.pitch_tinypeak_index:
            timeline[each] = 'TINY_PEAK' 
        for each in self.pitch_peak_index:
            timeline[each] = 'PEAK'

        f = open('log.csv', 'a')
        st = ''
        for i in range(len(timeline)):
            if timeline[i] == 'NEAR_ZERO':
                st = '1'
            elif timeline[i] == 'VALLEY':
                st = '2'
            elif timeline[i] == 'TINY_PEAK':
                st = '3'
            elif timeline[i] == 'PEAK':
                st = '4'
            else:
                st = '0'
            out = str(i)+","+st+"\n"
            f.writelines(out)

        self.timeline = timeline
        f.close()

    def seperatetimeline(self):
        timeline = self.timeline
        self.route = []
        count = 0
        count_buffer = Pitch_period()
        
        for i in range(len(timeline)):
            if timeline[i] == 'VALLEY':
                if count == 0 : 
                    count = 1
                    count_buffer.start_valley = i
                elif count == 3:
                    count = 1
                    count_buffer.end_valley = i
                    self.route.append(copy.copy(count_buffer))
                    count_buffer.clear()
                    count_buffer.start_valley = i
                else:
                    count = 0
                    count_buffer.clear()
            elif timeline[i] == 'PEAK':
                if count == 1:
                    count = 2
                    count_buffer.peak = i
                else:
                    count = 0
                    count_buffer.clear()
            elif timeline[i] == 'NEAR_ZERO' and count==2:
                count_buffer.nearzero.append(i)
            elif timeline[i] == 'TINY_PEAK':
                if count == 2:
                    count = 3
                    count_buffer.tiny_peak = i
                else:
                    count = 0
                    count_buffer.clear()
        self.timeline = timeline


    def countstep(self):
        t = 0.0
        for each in self.route:
            t += (each.end_valley-each.start_valley)
        t = t/len(self.timeline)
        print('step is',len(self.route)/t)
        printable_route = []
        for each in self.route:
            printable_route.extend(each.tolist())
        return printable_route

    def supportrate(self):
        sum,n = 0,len(self.route)
        for each in self.route:
            # if len(each.nearzero)>1:
            sum += (each.tiny_peak-each.peak)/(each.end_valley-each.start_valley)
            # else:
            #     n -= 1
        r = round(sum/n*100,2)
        print('supportrate is',r,'%')

    def steplength(self):
        sum,n = 0,0
        for i in range(1,len(self.route)):
            if self.route[i-1].end_valley == self.route[i].start_valley:
                sum += self.Sx[self.route[i].nearzero[0]]-self.Sx[self.route[i-1].nearzero[0]]
                n +=1
        avr = sum/n
        print('avr steplength is',avr,'m')
    
    def rollangle(self):
        sum,n = 0,0
        for each in self.route:
            index = each.nearzero[0]
            sum += self.roll[index]
            n += 1
        avr = sum/n/math.pi*180
        print('avr rollangle is',avr,'°')

