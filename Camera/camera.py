import cv2
import time

class Camera:
    def cam_names(self):
        return ['peanuts']
        
    def cam_init(self, camera_index = [0]):
        self.camera_index = camera_index
        self.cameras = []
        for i in range(len(camera_index)):
            cap = cv2.VideoCapture(camera_index[i])
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)

            if cap.isOpened():
                self.cameras.append({"id": camera_index[i], "cap": cap})
                print(f"Camera_{self.cam_names()[i]} initialized.") 
            else:
                raise Exception(f"Warning: Camera_{self.cam_names()[i]} could not be opened.")

    def capture(self, save_images=False, flush=True, flush_grabs=5, flush_timeout_ms=60):
        frames = []
        for i, cam in enumerate(self.cameras):
            cap = cam["cap"]

            # 先丟掉緩衝中的舊影像（以張數或時間為上限）
            if flush:
                start = time.time()
                for n in range(flush_grabs):
                    cap.grab()
                    if (time.time() - start) * 1000 >= flush_timeout_ms:
                        break
            # 正式讀取最新影像
            ret, frame = cap.read()
            if ret:
                if save_images:
                    filename = f"camera_{self.cam_names()[i]}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Saved frame from Camera_{self.cam_names()[i]} to {filename}")

                frames.append(frame)
            else:
                raise Exception(f"Warning: Camera_{self.cam_names()[i]} failed to read frame.")
        return frames
    
    def capture_single(self, cam_index, save_image=False, flush=True, flush_grabs=5, flush_timeout_ms=60):
        for i, cam in enumerate(self.cameras):
            if self.camera_index[i] != cam_index:
                continue

            cap = cam["cap"]

            # 先丟掉緩衝中的舊影像（以張數或時間為上限）
            if flush:
                start = time.time()
                for n in range(flush_grabs):
                    cap.grab()
                    if (time.time() - start) * 1000 >= flush_timeout_ms:
                        break
            # 正式讀取最新影像
            ret, frame = cap.read()
            if ret:
                if save_image:
                    filename = f"camera_{self.cam_names()[i]}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"Saved frame from Camera_{self.cam_names()[i]} to {filename}")

                return frame
            else:
                raise Exception(f"Error: Camera_{self.cam_names()[i]} failed to read frame.")
            
        raise Exception(f"Error: cam_index not in cameras.")

    def quit(self):
        for i, cam in enumerate(self.cameras):
            cam["cap"].release()
            print(f"Camera_{self.cam_names()[i]} released.")

# ===== 使用範例 =====
def main():
    cam = Camera()
    cam.cam_init([4,6,2,0])
    #for i in range(10):
    while True:
        frames=cam.capture(True)
        time.sleep(0.5)
        #print(f"Captured frames: {len(frames)}")
        #print(f"time_eslapsed: {time.time()}")
    cam.quit()

if __name__ == '__main__':
    main()
