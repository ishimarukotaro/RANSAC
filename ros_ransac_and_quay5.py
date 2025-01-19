#!/usr/bin/env python3

from __future__ import print_function

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import random
import math
from collections import deque
# メッセージの型等のimport
from sensor_msgs.msg import PointCloud2
from shipcon_pcc.msg import ransac_and_quay_msgs
from shipcon_pcc.msg import lidar2

class Publishsers():
    def __init__(self):
        self.pub_ransac = rospy.Publisher('ransac_and_quay', ransac_and_quay_msgs, queue_size=10)
        self.msg_ransac = ransac_and_quay_msgs()
        # Publisherを作成
        self.pub_lidar = rospy.Publisher('lidar2', lidar2, queue_size=10)
        self.msg_lidar = lidar2()
        # messageの型を作成

    def make_msg(self, RANSAC, best_params, psi, d_Lm, rotate_velo, sway_velo, min_params, psi_median, d_Lm_median, rotate_velo_median, sway_velo_median, ransac_time): 
        # if best_params.shape == (3,1):
        #     best_params = np.append(best_params, np.array([[best_params[0,0]], [best_params[1,0]], [best_params[2,0]]]), axis=1) 
        # self.msg_ransac.params_a = best_params[0,:]
        # print('unko'+str(self.msg_ransac.params_a))
        # self.msg_ransac.params_b = best_params[1,:]
        # print('uncho'+str(self.msg_ransac.params_b))
        # self.msg_ransac.params_c = best_params[2,:]
        self.msg_ransac.psi = psi
        self.msg_ransac.d_Lm = d_Lm
        self.msg_ransac.rotate_velo = rotate_velo
        self.msg_ransac.sway_velo = sway_velo
        self.msg_ransac.psi_median = psi_median
        self.msg_ransac.d_Lm_median = d_Lm_median
        self.msg_ransac.rotate_velo_median = rotate_velo_median
        self.msg_ransac.sway_velo_median = sway_velo_median
        self.msg_ransac.quay_params = min_params
        self.msg_ransac.ransac_time = ransac_time
        self.msg_ransac.best_num = RANSAC.best_num
        self.msg_ransac.thresh = RANSAC.thresh
        self.msg_ransac.maxIteration1 = RANSAC.maxIteration1
        self.msg_ransac.maxIteration2 = RANSAC.maxIteration2
        self.msg_ransac.smpl_distance = RANSAC.smpl_distance
        self.msg_ransac.min_samples = RANSAC.min_samples
        self.msg_ransac.points_0 = RANSAC.points_0
        # self.msg_ransac.points_1 = RANSAC.points_1
        # self.msg_ransac.points_2 = RANSAC.points_2
        # self.msg_ransac.points_4 = RANSAC.points_4
        # self.msg_ransac.points_6 = RANSAC.points_6
        # self.msg_ransac.points_8 = RANSAC.points_8
        # self.msg_ransac.points_10 = RANSAC.points_10
        # self.msg_ransac.points_12 = RANSAC.points_12
        # self.msg_ransac.points_14 = RANSAC.points_14
        self.msg_lidar.psi = psi
        self.msg_lidar.d_Lm = d_Lm
        self.msg_lidar.rotate_velo = rotate_velo
        self.msg_lidar.sway_velo = sway_velo
        self.msg_lidar.psi_median = psi_median
        self.msg_lidar.d_Lm_median = d_Lm_median
        self.msg_lidar.rotate_velo_median = rotate_velo_median
        self.msg_lidar.sway_velo_median = sway_velo_median

    def send_msg(self, msg_ransac, msg_lidar, RANSAC, best_params, psi, d_Lm, rotate_velo, sway_velo, min_params, psi_median, d_Lm_median, rotate_velo_median, sway_velo_median, ransac_time):
        self.make_msg(RANSAC, best_params, psi, d_Lm, rotate_velo, sway_velo, min_params, psi_median, d_Lm_median, rotate_velo_median, sway_velo_median, ransac_time)
        self.pub_ransac.publish(msg_ransac)
        self.make_msg(RANSAC, best_params, psi, d_Lm, rotate_velo, sway_velo, min_params, psi_median, d_Lm_median, rotate_velo_median, sway_velo_median, ransac_time)
        self.pub_lidar.publish(msg_lidar)

class Subscribers():
    def __init__(self):
        #importしたメッセージファイルを箱に代入
        self.PointCloud2 = PointCloud2()  
        # LiDAR 
        self.velodyne_points = rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_cb)

    def pointcloud_cb(self, msg):
        self.PointCloud2 = msg

class ransac():
    def __init__(self):
        self.dist_list = []

        self.thresh = 0.1
        self.maxIteration1 = 100.0
        self.maxIteration2 = 50.0
        self.smpl_distance = 2.0

    def refrsh_data(self, sub):     #センサからのデータを常に更新する
        ## processed point cloud data ##
        cloud = pc2.read_points(sub.PointCloud2, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True)
        filtered_points = []
        for point in cloud:
            x, y, z, ring, intensity = point
            
            if (-0.8 > x) or (x > 2.7) or (-0.41 > y) or (y > 0.41): # for delete model ship points
                r = math.sqrt(x ** 2 + y ** 2)
                if (8 > r):                             # for delete bamboo bush and structure
                    filtered_points.append((x, y, z))
        
        x_list = []
        y_list = []
        z_list = []
        for p in filtered_points:
            x_list.append(p[0])
            y_list.append(p[1])
            z_list.append(p[2])
        self.xyz_arr = np.array([x_list, y_list, z_list]) #(3,N)
        self.xyz_arr_trans = self.xyz_arr.T

        self.processed_points = self.xyz_arr[0:2, :] #(2, N)
        self.points_0 = self.processed_points.shape[1]

    # def pre_processing(self): #計算速度向上のため点群を前処理
    #     x = self.all_array[0, :]
    #     y = self.all_array[1, :]
    #     no_x_condition = np.where((-0.8 < x) & (x < 2.7))[0] #xをdeleteする条件(模型船内の点群)
    #     no_y_condition = np.where((-0.41< y) & (y < 0.41))[0] #yをdeleteする条件(模型船内(B+1)の点群)
    #     # no_y_condition = np.where(-0.41< y)[0] #yをdeleteする条件(模型船内(B+0.1m)の点群)
    #     no_ship_condition = np.intersect1d(no_x_condition, no_y_condition) #x,yどちらも満たす条件
    #     no_ship_points = np.delete(self.all_array, no_ship_condition, axis=1)

    #     x = no_ship_points[0, :]
    #     y = no_ship_points[1, :]
    #     quay_condition = np.where((0 < y) | (y < -8) | (x > -8))[0] #y>0,つまり池固定座標系において建物がある方を消去する。ただ、lidarのデータは船体固定座標系なので、おおよそ岸壁ではない方の点群を消去している
    #     #また、竹やぶのほうが意外と真っ直ぐに並んでいて多くの点群があるため、こちらの点群も消去した（２つ目の条件）
    #     self.processed_points = np.delete(no_ship_points, quay_condition, axis=1)[0:2,:] #(xy)の(2,M)の行列
        
    #     # z = quay_points[2, :]
    #     # z_condition = np.where(z>5)[0] #zのdeleteする条件
    #     # no_z_points = np.delete(quay_points, z_condition, axis=1)

    #     # intensity = no_z_points[3, :]
    #     # intensity_condition = np.where(intensity<=20)[0] #intensityのdeleteする条件
    #     # processed_points = np.delete(no_z_points, intensity_condition, axis=1)
    #     # self.processed_points = np.delete(no_z_points, intensity_condition, axis=1)[0:2,:]

    #     # ring = processed_points[4, :]
    #     # #仰角が正のものは１を除き消去する
    #     # ring_condition9 = (ring==9)
    #     # ring_condition10 = (ring==10)
    #     # ring_condition11 = (ring==11)
    #     # ring_condition12 = (ring==12)
    #     # ring_condition13 = (ring==13)
    #     # ring_condition14 = (ring==14)
    #     # ring_condition15 = (ring==15)
    #     # comb_ring_cond = ring_condition9 | ring_condition10 | ring_condition11 | ring_condition12 | ring_condition13 | ring_condition14 | ring_condition15
    #     # ring_condition = np.where(comb_ring_cond)[0] #ringのdeleteする条件
    #     # self.processed_points = np.delete(processed_points, ring_condition, axis=1)[0:2,:] #(xy)の(2,M)の行列
    #     self.points_0 = self.processed_points.shape[1]

    def fit(self, pts, best_params, thresh=None, maxIteration1=None, maxIteration2=None, smpl_distance=None): #RANSACのメイン
        """
        Find the best equation for the 2D line. The line in a 2d(xy) enviroment is defined as ax+by+c=0.

        :param pts: 2D(xy) point cloud as a `np.array (N,2)`.
        :param thresh: Threshold distance from the line which is considered inlier.
        :param maxIteration: Number of maximum iteration which RANSAC will loop over.
        :returns:
        - best_params: the best a,b,c np.array(1,3)
        ---
        """
        if thresh is None and maxIteration1 is None and maxIteration2 is None and smpl_distance is None:
            thresh = self.thresh
            maxIteration1 = self.maxIteration1
            maxIteration2 = self.maxIteration2
            smpl_distance = self.smpl_distance

        try:
            self.min_samples = math.floor(self.points_0*0.5) # 全ての点群に割合をかけている。この割合は経験則。小数点の切り捨て
        except:
            self.min_samples = 900                          # 一番はじめはno attributeなのでtry, exceptで場合分けしている

        n_points = pts.shape[0]
        outer_iterations = 0
        inner_iterations = 0
        loss_list = []
        inliers_num_list = []
        params_array = np.empty((3,0))
        samples_id_array = np.empty((2,0), dtype=int)

        while outer_iterations < maxIteration1:
            while inner_iterations < maxIteration2:
                # Samples 2 random points
                id_samples = random.sample(range(0, n_points), 2)
                pt_samples = pts[id_samples]
                x1 = pt_samples[0,0]
                y1 = pt_samples[0,1]
                x2 = pt_samples[1,0]
                y2 = pt_samples[1,1]
                sample_distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
                if sample_distance <= smpl_distance:
                    inner_iterations += 1
                else:
                    id_samples = id_samples
                    break

            # calcurate a,b,c with pt_samples
            p = pt_samples[0]
            q = pt_samples[1]
            a_smpl = p[1] - q[1]
            b_smpl = q[0] - p[0]
            c_smpl = p[0] * q[1] - q[0] * p[1]
            params_smpl = np.array([[a_smpl], [b_smpl], [c_smpl]])

            # Distance from a point to a line
            # http://www.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/tech0044.html
            # P = Q0 + ((P1-Q0)*v)*v
            # dist_pt = ||P-pts||
            Q0 = np.array([x1, y1])
            Q1 = np.array([x2, y2])
            v = np.array(Q1-Q0)/np.linalg.norm(Q1-Q0)
            v_size = v.shape[0]
            tmp = (pts - Q0) @ v
            tmp_size = tmp.shape[0]
            P = Q0 + tmp.reshape(tmp_size,1) @ v.reshape(1,v_size)
            dist_pt = np.linalg.norm(P - pts, axis=1)

            # Select indexes where distance is smaller than the threshold
            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            inliers_num = pt_id_inliers.size

            # calcurate loss
            tmp_2 = (pts[pt_id_inliers] - Q0) @ v
            tmp_2_size = tmp_2.shape[0]
            P_2 = Q0 + tmp_2.reshape(tmp_2_size,1) @ v.reshape(1,v_size)
            dist_inlier = np.linalg.norm(P_2 - pts[pt_id_inliers], axis=1) # distance b/w inlier points and line
            loss = np.sum(dist_inlier)

            if len(pt_id_inliers) > self.min_samples:
                loss_list.append(loss)
                inliers_num_list.append(inliers_num)
                params_array = np.append(params_array, params_smpl, axis=1)
                samples_id_array = np.append(samples_id_array, np.array([[id_samples[0]],[id_samples[1]]]), axis=1)
                
            outer_iterations += 1
        
        if len(loss_list) !=0:
            best_index = np.argmin(loss_list)
            self.best_num = inliers_num_list[best_index]
            best_param = params_array[:, best_index]
            best_params = np.append(best_params, np.array([[best_param[0]],[best_param[1]],[best_param[2]]]), axis=1)
            # 複数本直線を求めるために、best_indexのときのsample_idを求める
            best_id = samples_id_array[:, best_index]
            best_samples = pts[best_id]
            R0 = np.array([best_samples[0,0], best_samples[0,1]])
            R1 = np.array([best_samples[1,0], best_samples[1,1]])
            # 真の直線のinlierを求める
            w = np.array(R1-R0)/np.linalg.norm(R1-R0)
            w_size = w.shape[0]
            tmp_w = (pts - R0) @ w
            tmp_w_size = tmp_w.shape[0]
            S = R0 + tmp_w.reshape(tmp_w_size,1) @ w.reshape(1,w_size)
            dist_pt_best = np.linalg.norm(S - pts, axis=1)

            # Select indexes where distance is bigger than the threshold
            pt_id_outliers_best = np.where(np.abs(dist_pt_best) > thresh)[0]
            outliers = pts[pt_id_outliers_best]

            self.processed_points = outliers
        else:
            best_param = np.array([0, 0, 0], dtype='float32') #船体から遠い直線とすることで、複数ある直線の中で岸壁には選ばれないようa,b,cを設定した。ただし、複数の直線がすべてこうなると問題である
            best_params = np.append(best_params, np.array([[best_param[0]],[best_param[1]],[best_param[2]]]), axis=1)
            self.best_num = 0
            #elseとなるということは適切な直線が引けないということなので、複数本の直線が上のbest_paramsになっても大丈夫
            print("unchi1")
            self.processed_points = pts
        
        return best_params

    def dist(self, params, p=np.array([1.01, 0])): #pはvelodyneから見たmidshipの座標＋重心の位置(今はmidship=重心としている)
        a = params[0]
        b = params[1]
        c = params[2]
        x = p[0]
        y = p[1]
        norm = np.linalg.norm([a, b])
        dist = np.abs(a*x + b*y + c) / norm
        self.dist_list.append(dist)

    def detect(self, params, p=np.array([1.01, 0])): #pはvelodyneから見たmidshipの座標＋重心の位置(今はmidship=重心としている)
        a = params[0]
        b = params[1]
        c = params[2]
        x = p[0]
        y = p[1]
        norm = np.linalg.norm([a, b])
        dist = np.abs(a*x + b*y + c) / norm
        if b == 0:
            psi = 0
        else:
            alpha = math.atan(-a/b) #x軸と直線がなす角
            # alphaは, 0 < alpha < pi, であり、psiは, -pi < psi < pi, であるのでその変換を行っている
            if alpha < math.pi/2:
                psi = alpha
            else:
                psi = alpha - math.pi 
        d_L = dist/math.cos(psi)
        return psi, d_L    

    # psi の値によってそのbest_paramsを消去する
    def psi_condition(self, best_params, psi_lim=60, p=np.array([1.01, 0])): #psi_limは(degree)        #pはvelodyneから見たmidshipの座標＋重心の位置(今はmidship=重心としている)
        psi_list = []
        psi_delete_list = []
        for i in range(best_params.shape[1]):
            psi = self.detect(best_params[:,i], p)
            psi_list.append(psi[0])
        for j in range(best_params.shape[1]):
            if abs(psi_list[j]) > psi_lim/180*math.pi:
                psi_delete_list.append(j)
        best_params = np.delete(best_params, psi_delete_list, 1)
        return best_params

    #https://qiita.com/zaki858/items/c34537ec71e7c03a38e1このサイトより並び替えのアルゴリズムを持ってきた
    #RANSACのなまちだとブレることがあるのでmerge_sort(),merge()はメディアンフィルタの並び替えの部分として使用している
    def merge_sort(self, lst, RANSAC):
        if len(lst) <= 1:
            return lst

        mid = len(lst) // 2
        left = lst[:mid]
        right = lst[mid:]

        left = RANSAC.merge_sort(left, RANSAC)
        right = RANSAC.merge_sort(right, RANSAC)

        return self.merge(left, right)

    def merge(self, left, right):
        merged = []
        left_i, right_i = 0, 0

        while left_i < len(left) and right_i < len(right):
            if left[left_i] <= right[right_i]:
                merged.append(left[left_i])
                left_i += 1
            else:
                merged.append(right[right_i])
                right_i += 1

        if left_i < len(left):
            merged.extend(left[left_i:])
        if right_i < len(right):
            merged.extend(right[right_i:])
        
        return merged

def main():
    # nodeの立ち上げ
    rospy.init_node('NODE_ransac_and_quay', anonymous = True)
    # インスタンスの作成と実行
    pub = Publishsers()
    sub = Subscribers()
    # 速度を求めるためにpsi, d_Lmを保存
    que_len = 10
    psi_que = deque([], que_len)
    d_Lm_que = deque([], que_len)
    time_que = deque([], que_len)
    #medianフィルタのかかったque
    filtered_psi_que = deque(que_len*[0], que_len)
    filtered_d_Lm_que = deque(que_len*[0], que_len)
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # インスタンスの作成と実行(dist_listに数値が蓄積されないように、毎回定義し直す)
        RANSAC = ransac()
        # refrsh関数の実行
        RANSAC.refrsh_data(sub)

        if RANSAC.xyz_arr.size == 0:
            print("unchi2")
            continue
        start_time = rospy.Time.now()
        start_time = start_time.to_sec()
        # 岸壁候補の直線の数
        line_num = 0
        # #pre_processing関数の実行
        # RANSAC.pre_processing()
        #best_paramsの箱
        best_params = np.empty((3,0))
        #fit関数の実行
        if RANSAC.processed_points.shape[1] > 1 :
            best_params = RANSAC.fit(RANSAC.processed_points.T, best_params)
            line_num += 1
            # print(RANSAC.best_num)
            # if RANSAC.processed_points.shape[0] > 1:
            #     best_params = RANSAC.fit(RANSAC.processed_points, best_params)
            #     line_num += 1
            #     print(RANSAC.best_num)
            #     if RANSAC.processed_points.shape[0] > 1:
            #         best_params = RANSAC.fit(RANSAC.processed_points, best_params)
            #         line_num += 1
            #         if RANSAC.processed_points.shape[0] > 1:
            #             best_params = RANSAC.fit(RANSAC.processed_points, best_params)
            #             line_num += 1
        else:
            print("unchi3")
            continue
        # psiの値で不適切なbest_paramsを省く
        # best_params = RANSAC.psi_condition(best_params)
        # if best_params.size == 0:
        #     continue
        # dist関数の実行
        for n in range(best_params.shape[1]):
            RANSAC.dist(best_params[:,n])
        #距離が一番小さいparamを求める
        min_index = RANSAC.dist_list.index(min(RANSAC.dist_list))
        min_params = best_params[:, min_index]
        psi, d_Lm = RANSAC.detect(min_params)
        psi_que.append(psi)
        d_Lm_que.append(d_Lm)
        time_que.append(start_time)
        # 速度を求めるための作業
        print(line_num)
        if len(psi_que) == que_len:
            rotate_velo = -(psi_que[que_len-1] - psi_que[0])/(time_que[que_len-1] - time_que[0])
            sway_velo = -(d_Lm_que[que_len-1] - d_Lm_que[0])/(time_que[que_len-1] - time_que[0])

            #メディアンフィルタ
            sort_psi_que = RANSAC.merge_sort([x for x in psi_que], RANSAC)
            sort_d_Lm_que = RANSAC.merge_sort([x for x in d_Lm_que], RANSAC)
            psi_median = np.median(sort_psi_que)
            d_Lm_median = np.median(sort_d_Lm_que)
            filtered_psi_que.append(psi_median)
            filtered_d_Lm_que.append(d_Lm_median)
            rotate_velo_median = -(filtered_psi_que[que_len-1] - filtered_psi_que[0])/(time_que[que_len-1] - time_que[0])
            sway_velo_median = -(filtered_d_Lm_que[que_len-1] - filtered_d_Lm_que[0])/(time_que[que_len-1] - time_que[0])
        else:
            print("unchi4")
            continue 

        end_time = rospy.Time.now()
        end_time = end_time.to_sec()
        ransac_time = end_time - start_time
        # print(ransac_time)
        pub.send_msg(pub.msg_ransac, pub.msg_lidar, RANSAC, best_params, psi, d_Lm, rotate_velo, sway_velo, min_params, psi_median, d_Lm_median, rotate_velo_median, sway_velo_median, ransac_time)
        
        #実行周波数を維持する?
        rate.sleep()

if __name__ == '__main__':
   main()