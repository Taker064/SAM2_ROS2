#画像同期・条件指定なしのテストファイル.case1とほぼ同じ
import sys
sys.path.append('/work')#SAM2のファイル先にパスをつなげている
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #ROS2ImageとOpenCVをつなげるライブラリ
import cv2 #OpenCVライブラリ
import numpy as np
import time
import os
import torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor


class Sam2_Rostest(Node):
    def __init__(self):
        super().__init__("realsense_subscriber_python")
        self.declare_parameter('image_topic_name', '/camera/camera/color/image_raw') #パラメータの宣言,launchでパラメータの設定変えた場合はトピック名を変える必要あり
        image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value #宣言したパラメータからパラメータ内の値を取得し、string型に変換

        video_qos = rclpy.qos.QoSProfile(depth=10) #QOSの設定,要は通信品質のやつ
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT #BEST_EFFORTはやり取りするデータに欠損などが発生しても送る。初期設定だとRELIABLEになっていてデータが送られないらしい

        self.sub_img = self.create_subscription(#Image型,topic名image_topicnameのtopicを受け取る。通信品質は上記のvideo_qosに従う
            Image,
            image_topic_name,
            self.on_image_subscribed,
            video_qos
        )
        self.publisher = self.create_publisher(#publisher設定.OpenCV画像をROStiopicに変更して送信する
            Image,
            'test',
            10
        )
        #チェックポイントとyamlの指定
        checkpoint = "/work/checkpoints/sam2.1_hiera_large.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"

        #モデルの初期化
        self.predictor = SAM2ImagePredictor(build_sam2(model_cfg, checkpoint))

        self.lasttime = time.time() 


        self.sub_img
    
    def on_image_subscribed(self, img):#コールバック関数
        start = time.time()

        if start - self.lasttime >= 2.0:
            img_np = CvBridge().imgmsg_to_cv2(img)#realsenseから受け取った画像をOpenCVで使う画像に変換
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)#カラー画像に変換
            demo_img = img_np.copy()
            gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)#カラー画像をグレースケール画像へ変換

            #セグメンテーション処理,マスク画像の生成
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                self.predictor.set_image(img_np)
                masks, _, _ = self.predictor.predict(
                point_coords = np.array([[300,450]]),
                point_labels = np.array([1]),
                multimask_output = False,
            )   

            #マスク処理
            mask = masks[0]
            mask = mask.astype(bool)
            color = (255, 0, 0)
            colored_mask = np.zeros_like(img_np)
            colored_mask[mask] = np.array(color)
            demo_img = cv2.addWeighted(demo_img, 1, colored_mask, 0.8, 0)

            #セグメンテーション済画像をTopicで送信
            result_msg = CvBridge().cv2_to_imgmsg(demo_img, 'passthrough')
            self.publisher.publish(result_msg)

            #待機時間の更新
            self.lasttime = start 

            cv2.waitKey(1)#キー入力を待機


def main(args=None):
    try:
        rclpy.init(args=args)#初期化
        rclpy.spin(Sam2_Rostest())#ノードを起動
    
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()#Ctrl+cで終了
        

if __name__ == "__main__":#Pythonスクリプトが直接実行された場合にのみにmain関数を実行
    main()