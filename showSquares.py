import csv
import numpy as np
from matplotlib import pyplot as plt
from numpy import cos, sin, pi, sqrt, arctan2, arccos
from copy import deepcopy

def readMR18DataFromCSV(file_path):
    # Initialize an empty list to store data
    data = []

    # Open the CSV file and read its content using csv.DictReader
    with open(file_path, newline='') as csvfile:
        # Create a DictReader object
        csv_reader = csv.DictReader(csvfile)

        # Iterate over rows and extract data
        for row in csv_reader:
            data.append(row)

    # Extract column names from the header
    columns = list(data[0].keys())

    # Create a NumPy 2D array

    motor = np.array([float(row['state_machine.SM_St_log']) for row in data])
    indmstart = np.where(motor>12)[0][0]
    
    mr18Data = []
    mr18Data.append(np.array([float(row['mr18.m0']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m1']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m2']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m3']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m4']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m5']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m6']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m7']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m8']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m9']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m10']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m11']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m12']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m13']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m14']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m15']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m16']) for row in data]))
    mr18Data.append(np.array([float(row['mr18.m17']) for row in data]))
    mr18Data= np.array(mr18Data).transpose()

    mr18valid=[]
    mr18valid.append(np.array([float(row['mr18ve.valid0']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid1']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid2']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid3']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid4']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid5']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid6']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid7']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid8']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid9']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid10']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid11']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid12']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid13']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid14']) for row in data]))
    mr18valid.append(np.array([float(row['mr18ve.valid15']) for row in data]))
    mr18valid = np.array(mr18valid).transpose()

    mr18hullX = []
    mr18hullX.append(np.array([float(row['mr18ve.chullx0']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx1']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx2']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx3']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx4']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx5']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx6']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx7']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx8']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx9']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx10']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx11']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx12']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx13']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx14']) for row in data]))
    mr18hullX.append(np.array([float(row['mr18ve.chullx15']) for row in data]))
    mr18hullX = np.array(mr18hullX).transpose()

    mr18hullY = []
    mr18hullY.append(np.array([float(row['mr18ve.chully0']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully1']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully2']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully3']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully4']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully5']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully6']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully7']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully8']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully9']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully10']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully11']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully12']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully13']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully14']) for row in data]))
    mr18hullY.append(np.array([float(row['mr18ve.chully15']) for row in data]))
    mr18hullY = np.array(mr18hullY).transpose()

    mr18quadX = []
    mr18quadX.append(np.array([float(row['mr18ve.bquadx0']) for row in data]))
    mr18quadX.append(np.array([float(row['mr18ve.bquadx1']) for row in data]))
    mr18quadX.append(np.array([float(row['mr18ve.bquadx2']) for row in data]))
    mr18quadX.append(np.array([float(row['mr18ve.bquadx3']) for row in data]))
    mr18quadX = np.array(mr18quadX).transpose()

    mr18quadY = []
    mr18quadY.append(np.array([float(row['mr18ve.bquady0']) for row in data]))
    mr18quadY.append(np.array([float(row['mr18ve.bquady1']) for row in data]))
    mr18quadY.append(np.array([float(row['mr18ve.bquady2']) for row in data]))
    mr18quadY.append(np.array([float(row['mr18ve.bquady3']) for row in data]))
    mr18quadY = np.array(mr18quadY).transpose()

    mr18hullSX = np.array([float(row['mr18ve.hullSX']) for row in data])
    mr18hullSY = np.array([float(row['mr18ve.hullSY']) for row in data])

    mr18quadSX = np.array([float(row['mr18ve.quadSX']) for row in data])
    mr18quadSY = np.array([float(row['mr18ve.quadSY']) for row in data])

    mr18hullCX = np.array([float(row['mr18ve.hullX']) for row in data])
    mr18hullCY = np.array([float(row['mr18ve.hullY']) for row in data])

    mr18quadCX = np.array([float(row['mr18ve.quadX']) for row in data])
    mr18quadCY = np.array([float(row['mr18ve.quadY']) for row in data])

    mr18quad = np.array([mr18quadCX, mr18quadCY]).transpose()
    mr18hull = np.array([mr18hullCX, mr18hullCY]).transpose()
    mr18hullS = np.array([mr18hullSX, mr18hullSY]).transpose()
    mr18quadS = np.array([mr18quadSX, mr18quadSY]).transpose()

    return mr18Data[indmstart:]*.001, mr18valid[indmstart:], mr18hullX[indmstart:], mr18hullY[indmstart:], \
           mr18quadX[indmstart:], mr18quadY[indmstart:], mr18hull[indmstart:], mr18quad[indmstart:], \
           mr18hullS[indmstart:], mr18quadS[indmstart:]

def __main__():
    # Specify the file path
    file_path = "/home/valentin/crazyflie/Recordings/tube/sd_33_cf2412.csv"

    mr18Data, mr18valid, mr18hullX, mr18hullY, mr18quadX, mr18quadY, mr18hull, mr18quad, mr18hullS, mr18quadS = readMR18DataFromCSV(file_path)
    
    mr18azimuth = np.array([0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5])

    datalen = len(mr18Data[:,0])
    for i in range(0,datalen,10):
        lengths = mr18Data[i, 0:16]
        valid = mr18valid[i, 0:16]
        indV = np.where(valid>0.5)
        indNV = np.where(valid<0.5)
        inds = np.array(range(1,48,3))
        indr = np.array(range(0,48,3))


        raysV = np.zeros((48, 2))+np.nan
        raysV[inds[indV], 0] = lengths[indV]*cos(np.deg2rad(mr18azimuth[indV]))
        raysV[inds[indV], 1] = lengths[indV]*sin(np.deg2rad(mr18azimuth[indV]))
        raysV[indr[indV], 0] = lengths[indV]*cos(np.deg2rad(mr18azimuth[indV]))*0
        raysV[indr[indV], 1] = lengths[indV]*sin(np.deg2rad(mr18azimuth[indV]))*0

        raysNV = np.zeros((48, 2))+np.nan
        raysNV[inds[indNV], 0] = lengths[indNV]*cos(np.deg2rad(mr18azimuth[indNV]))
        raysNV[inds[indNV], 1] = lengths[indNV]*sin(np.deg2rad(mr18azimuth[indNV]))
        raysNV[indr[indNV], 0] = lengths[indNV]*cos(np.deg2rad(mr18azimuth[indNV]))*0
        raysNV[indr[indNV], 1] = lengths[indNV]*sin(np.deg2rad(mr18azimuth[indNV]))*0

        # plt.plot(mr18hullX[i], mr18hullY[i], 'g', linewidth=1)
        quadX = np.concatenate((mr18quadX[i], [mr18quadX[i][0]]))
        quadY = np.concatenate((mr18quadY[i], [mr18quadY[i][0]]))
        plt.figure(1)
        plt.plot(raysV[:, 0], raysV[:, 1], 'b', linewidth=1)
        plt.plot(raysNV[:, 0], raysNV[:, 1], 'r', linewidth=1)
        plt.plot(quadX, quadY, 'm', linewidth=1)

        plt.plot(mr18hull[i, 0], mr18hull[i, 1], 'gx', linewidth=1)
        plt.plot(mr18quad[i, 0], mr18quad[i, 1], 'm+', linewidth=1)
        plt.plot(mr18hullS[i,0], mr18hullS[i, 1], 'm*', linewidth=1)
        plt.plot(mr18quadS[i, 0], mr18quadS[i, 1], 'g^', linewidth=1)
        plt.axis('Equal')
        plt.show()
        plt.figure(2)
        plt.subplot(221)
        plt.plot(quadX-mr18hull[i, 0], quadY-mr18hull[i, 1], 'm', linewidth=1)
        plt.axis('Equal')
        plt.subplot(222)
        plt.plot(quadX-mr18quad[i, 0], quadY-mr18quad[i, 1], 'g', linewidth=1)
        plt.axis('Equal')
        plt.subplot(223)
        plt.plot(quadX-mr18hullS[i,0], quadY-mr18hullS[i, 1], 'r', linewidth=1)
        plt.axis('Equal')
        plt.subplot(224)
        plt.plot(quadX-mr18quadS[i, 0], quadY-mr18quadS[i, 1], 'b', linewidth=1)
        plt.axis('Equal')
        plt.show()
        plt.figure(3)
        plt.subplot(221)
        plt.plot(raysV[:, 0]-mr18hull[i, 0], raysV[:, 1]-mr18hull[i, 1], 'm.', linewidth=1)
        plt.axis('Equal')
        plt.subplot(222)
        plt.plot(raysV[:, 0]-mr18quad[i, 0], raysV[:, 1]-mr18quad[i, 1], 'g.', linewidth=1)
        plt.axis('Equal')
        plt.subplot(223)
        plt.plot(raysV[:, 0]-mr18hullS[i,0], raysV[:, 1]-mr18hullS[i, 1], 'r.', linewidth=1)
        plt.axis('Equal')
        plt.subplot(224)
        plt.plot(raysV[:, 0]-mr18quadS[i, 0], raysV[:, 1]-mr18quadS[i, 1], 'b.', linewidth=1)
        plt.axis('Equal')
        plt.show()       
        aa=1
        None

        

if __name__ == "__main__":
    __main__()
