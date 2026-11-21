import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn     # 板子上有

# 后续改成detector
class Detector():
    def __init__(self,cam):
        self._MODEL = None
        self._MODEL_HEADS = None
        self._INPUT_W = 640
        self._INPUT_H = 640
        self._CLASS_NAMES = ["orange", "green", "black", "blue", "blue_home", "red_home", "home"]
        self.cam=cam    
        self.final_list=None
        self.mask=None
        self.roi_grey=None

    def _sigmoid(self,x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))

    def _init_engine(self,model_path):
        """
        内部函数：初始化模型并解析输出头
        """
        num_classes = len(self._CLASS_NAMES)
        self._MODEL = dnn.load(model_path)[0]
        
        in_shape = list(self._MODEL.inputs[0].properties.shape)
        if in_shape[1] == 3:  # NCHW
            self._INPUT_H, self._INPUT_W = int(in_shape[2]), int(in_shape[3])
        else:                 # NHWC
            self._INPUT_H, self._INPUT_W = int(in_shape[1]), int(in_shape[2])

        bbox_map, cls_map = {}, {}
        for idx, out in enumerate(self._MODEL.outputs):
            shp = list(out.properties.shape)
            if len(shp) != 4: continue
            
            if shp[1] == 64:       # NCHW
                bbox_map[(shp[2], shp[3])] = (idx, "NCHW")
            elif shp[-1] == 64:    # NHWC
                bbox_map[(shp[1], shp[2])] = (idx, "NHWC")
            elif shp[1] == num_classes and shp[2] > 1:
                cls_map[(shp[2], shp[3])] = (idx, "NCHW")
            elif shp[-1] == num_classes:
                cls_map[(shp[1], shp[2])] = (idx, "NHWC")

        heads = []
        for hw in sorted(bbox_map.keys(), key=lambda x: -x[0]):
            if hw not in cls_map: continue
            b_idx, b_fmt = bbox_map[hw]
            c_idx, c_fmt = cls_map[hw]
            stride = self._INPUT_H // hw[0]
            
            H, W = hw
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
            grid = np.stack([gx, gy], axis=-1).reshape(-1, 2).astype(np.float32)
            
            heads.append({
                "bbox_idx": b_idx, "cls_idx": c_idx, "hw": hw, 
                "stride": stride, "bbox_fmt": b_fmt, "cls_fmt": c_fmt, "grid": grid
            })
        self._MODEL_HEADS = heads


    def detect_all(self, conf=0.3, nms=0.45, 
    model_path="/home/sunrise/RS_workplace/models/26_1108/best_rdkx5.bin"):
        """
        输入图片、目标名称字符串（如 "redhome", "black"）,以及yolo识别的要求等。
        返回所有识别到的目标
        返回格式: list[[id,cx,cy,w,h],[...],...] 或 None
        """
        image=self.cam.img_for_detector
        if image is None:
            # print ("detect_all类没有获取到cam.img_for_detector")
            return None
        # 懒加载模型，如果不传入新模型就复用内存里的
        if self._MODEL is None:
            self._init_engine(model_path)
        orig_h, orig_w = image.shape[:2]
        
        # 前处理 (BGR -> NV12) 
        scale = min(self._INPUT_H / orig_h, self._INPUT_W / orig_w)
        new_h, new_w = int(orig_h * scale), int(orig_w * scale)
        resized = cv2.resize(image, (new_w, new_h))
        
        canvas = np.full((self._INPUT_H, self._INPUT_W, 3), 114, dtype=np.uint8)
        pad_top = (self._INPUT_H - new_h) // 2
        pad_left = (self._INPUT_W - new_w) // 2
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized

        yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y = yuv[:self._INPUT_H, :]
        u = yuv[self._INPUT_H:self._INPUT_H + self._INPUT_H // 4, :].reshape(self._INPUT_H // 2, self._INPUT_W // 2)
        v = yuv[self._INPUT_H + self._INPUT_H // 4:, :].reshape(self._INPUT_H // 2, self._INPUT_W // 2)
        uv = np.stack([u, v], axis=-1).reshape(self._INPUT_H // 2, self._INPUT_W)
        nv12 = np.concatenate([y, uv], axis=0)

        # 推理 
        outs = self._MODEL.forward(nv12)
        all_boxes, all_scores, all_cls = [], [], []

        # 后处理 (解码) 
        for h in self._MODEL_HEADS:
            H, W = h["hw"]
            stride = h["stride"]
            grid = h["grid"]
            reg_max = 16

            # 提取特征图
            b_buf = np.array(outs[h["bbox_idx"]].buffer, copy=False).astype(np.float32)
            bbox_feat = b_buf.reshape(1, 64, H, W).transpose(0, 2, 3, 1).reshape(-1, 64) if h["bbox_fmt"] == "NCHW" else b_buf.reshape(-1, 64)
            
            c_buf = np.array(outs[h["cls_idx"]].buffer, copy=False).astype(np.float32)
            cls_feat = c_buf.reshape(1, len(self._CLASS_NAMES), H, W).transpose(0, 2, 3, 1).reshape(-1, len(self._CLASS_NAMES)) if h["cls_fmt"] == "NCHW" else c_buf.reshape(-1, len(self._CLASS_NAMES))

            # 过滤置信度
            scores = self._sigmoid(cls_feat)
            max_scores = np.max(scores, axis=1)
            keep = max_scores >= conf
            if not np.any(keep): continue

            max_cls = np.argmax(scores, axis=1)
            bbox_keep = bbox_feat[keep]
            score_keep = max_scores[keep]
            cls_keep = max_cls[keep]
            grid_keep = grid[keep]

            # DFL 解码
            bbox = bbox_keep.reshape(-1, 4, reg_max)
            bbox_exp = np.exp(bbox - np.max(bbox, axis=-1, keepdims=True))
            bbox_sm = bbox_exp / np.sum(bbox_exp, axis=-1, keepdims=True)
            weights = np.arange(reg_max, dtype=np.float32).reshape(1, 1, -1)
            ltrb = np.sum(bbox_sm * weights, axis=-1)

            # 还原坐标
            xc = (grid_keep[:, 0] + 0.5) * stride
            yc = (grid_keep[:, 1] + 0.5) * stride
            x1 = np.clip((xc - ltrb[:, 0] * stride - pad_left) / scale, 0, orig_w)
            y1 = np.clip((yc - ltrb[:, 1] * stride - pad_top) / scale, 0, orig_h)
            x2 = np.clip((xc + ltrb[:, 2] * stride - pad_left) / scale, 0, orig_w)
            y2 = np.clip((yc + ltrb[:, 3] * stride - pad_top) / scale, 0, orig_h)

            areas = (x2 - x1) * (y2 - y1)
            valid = areas > 100
            if not np.any(valid): continue

            boxes = np.stack([x1[valid], y1[valid], x2[valid], y2[valid]], axis=1)
            all_boxes.append(boxes)
            all_scores.append(score_keep[valid])
            all_cls.append(cls_keep[valid])

        if not all_boxes:
            self.final_list = None
            return None

        boxes = np.concatenate(all_boxes)
        scores = np.concatenate(all_scores)
        classes = np.concatenate(all_cls)

        # NMS 去重
        # cv2.dnn.NMSBoxes 要求 [x, y, w, h]，且它不认类别，必须自己按类别分组
        xywh = np.stack([boxes[:, 0], boxes[:, 1],
                         boxes[:, 2] - boxes[:, 0],
                         boxes[:, 3] - boxes[:, 1]], axis=1)

        keep_idx = []
        for c in np.unique(classes):
            m = np.flatnonzero(classes == c)
            idx = cv2.dnn.NMSBoxes(xywh[m].tolist(), scores[m].tolist(), conf, nms)
            if idx is not None and len(idx) > 0:
                keep_idx.extend(m[np.array(idx).flatten()].tolist())

        keep_idx = np.array(sorted(keep_idx), dtype=int)
        if len(keep_idx) == 0:
            self.final_list = None
            return None

        final_boxes = boxes[keep_idx]
        # final_scores = scores[keep_idx]
        final_classes = classes[keep_idx]


        # 解包
        if len(final_boxes) > 0:
            # 1. 拆解 x1, y1, x2, y2(拆完也还是数组)
            x1 = final_boxes[:, 0]
            y1 = final_boxes[:, 1]
            x2 = final_boxes[:, 2]
            y2 = final_boxes[:, 3]

            # 2. 计算中心点 (cx, cy) 和 宽高 (w, h)
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w  = x2 - x1
            h  = y2 - y1

            # 3. 把类别 ID 一起拼进去，按列合并成 (N, 5) 的矩阵
            # 因为 final_classes 已经是整数，cx/cy/w/h 是浮点数，np.stack 会统一向上转型为浮点数
            result_array = np.stack([final_classes, cx, cy, w, h], axis=1)
            
            # 4. 如果你想强行把 id 变成整数（防止变成 4.0 这种浮点数），可以转换一下类型
            result_array[:, 0] = result_array[:, 0].astype(int)

            # 5. 转换为你要求的纯 Python List 格式
            self.final_list = result_array.tolist()
            for target in self.final_list:
                target[0]=self._CLASS_NAMES[int(target[0])]
        else:
            self.final_list = None
        return self.final_list

    def detect_closest(self, goal,conf=0.3, nms=0.45, 
    model_path="/home/sunrise/RS_workplace/models/26_1108/best_rdkx5.bin"):
        """
        这个函数负责筛选出最近的且正确的一个目标，输出list[id,cx,cy,w,h]或者None
        """
        my_list=[]
        security_list=[]
        block_list=[]
        _security=None
        approx=None
        final_list=self.detect_all(conf, nms, model_path)
        if final_list is None:
            return None
        for target in final_list:
            # 先筛出安全区--approx
            if target[0] in ["home","red_home","blue_home"]:
                security_list.append(target)
            elif target[0] in goal:
                block_list.append(target)
        if len(security_list)==0 or security_list is None:
            my_list=block_list
            # if target[0]==goal:
            #     my_list.append(target)
        else:
            max_area=None
            for security in security_list:
                _,_,_,w,h=security
                if max_area is None:
                    max_area=w*h
                    _security=security
                else:
                    if w*h>max_area:
                        max_area=w*h
                        _security=security
            _,cx,cy,w,h=_security
            approx=self.extract_home(cx,cy,w,h) # 这里大概率不会出现一边有图一边没图的结果
            if approx is None or len(approx)==0:
                my_list=final_list
            else:
                if "home" not in goal and "red_home" not in goal and "blue_home" not in goal :
                    for block in block_list:
                        _,cx,cy,_,_=block
                        result = cv2.pointPolygonTest(approx, (cx,cy), measureDist=False)
                        if result < 0:
                            my_list.append(block)
                else:
                    if len(security) ==0 or security is None:
                        return None
                    else:
                        for security in security_list:
                            id=security[0]
                            if id in goal:
                                my_list.append(security)
        minimum_distance=640000
        minimum_i=0
        if len(my_list) !=0:
            for i,target in enumerate(my_list):
                cx_=target[1]
                cy_=target[2]
                distance=cx_**2+cy_**2
                if distance <minimum_distance:
                    minimum_distance =distance
                    minimum_i=i
            return my_list[minimum_i]
        else:
            return None


    def extract_home(self,cx, cy, w, h):
        """
        输入图像，输入所识别的安全区中心点 cx, cy 和宽 w, 高 h；
        输出拟合出来的安全区角点 approx (4x2 的 numpy 数组) 或 None。
        """
        roi_padding=15
        # _list=self.detect_closest(goal=["home","red_home","blue_home"])
        image=self.cam.img_for_detector
        if image is None:
            # print ("detect_all类没有获取到cam.img_for_detector")
            return None
        # roi区域
        img_h, img_w = image.shape[:2]    # （480，640，3） 
        x1 = max(0, int(cx - w / 2) - roi_padding)  # 左上
        y1 = max(0, int(cy - h / 2) - roi_padding)
        x2 = min(img_w, int(cx + w / 2) + roi_padding)
        y2 = min(img_h, int(cy + h / 2) + roi_padding)
        
        # 如果裁剪区域无效，直接返回 None
        if x2 - x1 <= 0 or y2 - y1 <= 0:
            return None
            
        roi = image[y1:y2, x1:x2]
        roi_grey = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # 安全区
        _,self.mask=cv2.threshold(roi_grey,thresh=75,maxval=255,type=cv2.THRESH_BINARY_INV)
        kernel = np.ones((5, 5), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 所有的点都保留（可以试一下，效率慢就改SIMPLE）
        if not contours:
            return None
        largest_contour = max(contours, key=cv2.contourArea)
        # 计算轮廓周长
        peri = cv2.arcLength(largest_contour, True)
        # 用 2% 的精度去拟合多边形
        # 可以放宽精度再试一次，更多点
        approx = cv2.approxPolyDP(largest_contour, 0.02 * peri, True)   # 可调参
        # 将 4x1x2 的形状转换成 4x2 的形状
        approx = approx.reshape(-1, 2)
        # 还原点到原图
        for point in approx:
            point[0] += x1
            point[1] += y1
        self.roi_grey=roi_grey
        return approx

