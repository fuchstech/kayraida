import cv2
import numpy as np

class ColorFilter:
    def __init__(self):
        # HSV değerleri için başlangıç değerleri
        self.h_min = 100  # Mavi için preset
        self.h_max = 130
        self.s_min = 120
        self.s_max = 255
        self.v_min = 70
        self.v_max = 255
        
        # RGB değerleri için başlangıç değerleri (Mavi preset)
        self.r_min = 0
        self.r_max = 100
        self.g_min = 0
        self.g_max = 100
        self.b_min = 150
        self.b_max = 255
        
        # Filtreleme modu (True: HSV, False: RGB)
        self.use_hsv = True
        
        # Pencere isimleri
        self.window_name = "Orijinal Görüntü"
        self.filtered_window = "Filtrelenmiş Görüntü"
        self.control_window = "Renk Kontrolleri"
        
        # Kamera başlat
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            print("Kamera açılamadı! Varsayılan test görüntüsü kullanılacak.")
            self.use_camera = False
        else:
            self.use_camera = True
        
        self.setup_trackbars()
    
    def setup_trackbars(self):
        """HSV ve RGB değerlerini kontrol etmek için trackbar'ları oluştur"""
        cv2.namedWindow(self.control_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.control_window, 500, 600)
        
        # Mod seçici
        cv2.createTrackbar('Mode (0:HSV 1:RGB)', self.control_window, 0, 1, self.switch_mode)
        
        # HSV Trackbar'ları
        cv2.createTrackbar('H Min', self.control_window, self.h_min, 179, self.update_h_min)
        cv2.createTrackbar('H Max', self.control_window, self.h_max, 179, self.update_h_max)
        cv2.createTrackbar('S Min', self.control_window, self.s_min, 255, self.update_s_min)
        cv2.createTrackbar('S Max', self.control_window, self.s_max, 255, self.update_s_max)
        cv2.createTrackbar('V Min', self.control_window, self.v_min, 255, self.update_v_min)
        cv2.createTrackbar('V Max', self.control_window, self.v_max, 255, self.update_v_max)
        
        # RGB Trackbar'ları
        cv2.createTrackbar('R Min', self.control_window, self.r_min, 255, self.update_r_min)
        cv2.createTrackbar('R Max', self.control_window, self.r_max, 255, self.update_r_max)
        cv2.createTrackbar('G Min', self.control_window, self.g_min, 255, self.update_g_min)
        cv2.createTrackbar('G Max', self.control_window, self.g_max, 255, self.update_g_max)
        cv2.createTrackbar('B Min', self.control_window, self.b_min, 255, self.update_b_min)
        cv2.createTrackbar('B Max', self.control_window, self.b_max, 255, self.update_b_max)
        
        # Ön tanımlı renk filtreleri için butonlar
        cv2.createTrackbar('HSV Preset', self.control_window, 2, 4, self.apply_hsv_preset)  # Başlangıçta mavi
        cv2.createTrackbar('RGB Preset', self.control_window, 2, 4, self.apply_rgb_preset)  # Başlangıçta mavi
        
        # Bilgi etiketi oluştur
        self.create_info_window()
    
    def create_info_window(self):
        """Bilgi penceresi oluştur"""
        info_img = np.zeros((300, 450, 3), dtype=np.uint8)
        cv2.putText(info_img, 'RENK FILTRESI KONTROLLERI', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(info_img, 'Mode:', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(info_img, '0: HSV Filtreleme', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(info_img, '1: RGB Filtreleme', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.putText(info_img, 'HSV Preset Renkler:', (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(info_img, '0: Manuel  1: Kirmizi  2: Mavi', (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(info_img, '3: Yesil   4: Sari', (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        cv2.putText(info_img, 'RGB Preset Renkler:', (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(info_img, '0: Manuel  1: Kirmizi  2: Mavi', (10, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(info_img, '3: Yesil   4: Sari', (10, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        cv2.putText(info_img, 'ESC: Cikis', (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow('Bilgi', info_img)
    
    def switch_mode(self, mode):
        """HSV ve RGB modu arasında geçiş yap"""
        self.use_hsv = (mode == 0)
        print(f"Filtreleme modu: {'HSV' if self.use_hsv else 'RGB'}")
    
    # HSV Trackbar callback fonksiyonları
    def update_h_min(self, val):
        self.h_min = val
    
    def update_h_max(self, val):
        self.h_max = val
    
    def update_s_min(self, val):
        self.s_min = val
    
    def update_s_max(self, val):
        self.s_max = val
    
    def update_v_min(self, val):
        self.v_min = val
    
    def update_v_max(self, val):
        self.v_max = val
    
    # RGB Trackbar callback fonksiyonları
    def update_r_min(self, val):
        self.r_min = val
    
    def update_r_max(self, val):
        self.r_max = val
    
    def update_g_min(self, val):
        self.g_min = val
    
    def update_g_max(self, val):
        self.g_max = val
    
    def update_b_min(self, val):
        self.b_min = val
    
    def update_b_max(self, val):
        self.b_max = val
    
    def apply_hsv_preset(self, preset):
        """HSV ön tanımlı renk değerlerini uygula"""
        if preset == 1:  # Kırmızı
            self.set_hsv_values(0, 10, 120, 255, 70, 255)
        elif preset == 2:  # Mavi (varsayılan)
            self.set_hsv_values(100, 130, 120, 255, 70, 255)
        elif preset == 3:  # Yeşil
            self.set_hsv_values(40, 80, 120, 255, 70, 255)
        elif preset == 4:  # Sarı
            self.set_hsv_values(20, 35, 120, 255, 70, 255)
    
    def apply_rgb_preset(self, preset):
        """RGB ön tanımlı renk değerlerini uygula"""
        if preset == 1:  # Kırmızı
            self.set_rgb_values(150, 255, 0, 100, 0, 100)
        elif preset == 2:  # Mavi (varsayılan)
            self.set_rgb_values(0, 100, 0, 100, 150, 255)
        elif preset == 3:  # Yeşil
            self.set_rgb_values(0, 100, 150, 255, 0, 100)
        elif preset == 4:  # Sarı
            self.set_rgb_values(200, 255, 200, 255, 0, 100)
    
    def set_hsv_values(self, h_min, h_max, s_min, s_max, v_min, v_max):
        """HSV değerlerini ayarla ve trackbar'ları güncelle"""
        self.h_min, self.h_max = h_min, h_max
        self.s_min, self.s_max = s_min, s_max
        self.v_min, self.v_max = v_min, v_max
        
        # Trackbar'ları güncelle
        cv2.setTrackbarPos('H Min', self.control_window, h_min)
        cv2.setTrackbarPos('H Max', self.control_window, h_max)
        cv2.setTrackbarPos('S Min', self.control_window, s_min)
        cv2.setTrackbarPos('S Max', self.control_window, s_max)
        cv2.setTrackbarPos('V Min', self.control_window, v_min)
        cv2.setTrackbarPos('V Max', self.control_window, v_max)
    
    def set_rgb_values(self, r_min, r_max, g_min, g_max, b_min, b_max):
        """RGB değerlerini ayarla ve trackbar'ları güncelle"""
        self.r_min, self.r_max = r_min, r_max
        self.g_min, self.g_max = g_min, g_max
        self.b_min, self.b_max = b_min, b_max
        
        # Trackbar'ları güncelle
        cv2.setTrackbarPos('R Min', self.control_window, r_min)
        cv2.setTrackbarPos('R Max', self.control_window, r_max)
        cv2.setTrackbarPos('G Min', self.control_window, g_min)
        cv2.setTrackbarPos('G Max', self.control_window, g_max)
        cv2.setTrackbarPos('B Min', self.control_window, b_min)
        cv2.setTrackbarPos('B Max', self.control_window, b_max)
    
    def create_test_image(self):
        """Test görüntüsü oluştur (kamera yoksa)"""
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        
        # Renkli daireler çiz (BGR formatında)
        cv2.circle(img, (150, 150), 50, (0, 0, 255), -1)    # Kırmızı
        cv2.circle(img, (450, 150), 50, (255, 0, 0), -1)    # Mavi
        cv2.circle(img, (150, 300), 50, (0, 255, 0), -1)    # Yeşil
        cv2.circle(img, (450, 300), 50, (0, 255, 255), -1)  # Sarı
        cv2.circle(img, (300, 225), 30, (255, 0, 255), -1)  # Magenta
        cv2.circle(img, (300, 100), 25, (128, 128, 128), -1) # Gri
        cv2.circle(img, (300, 350), 25, (255, 255, 255), -1) # Beyaz
        
        return img
    
    def apply_hsv_filter(self, frame):
        """HSV renk filtresini uygula"""
        # BGR'dan HSV'ye dönüştür
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # HSV aralığını tanımla
        lower_bound = np.array([self.h_min, self.s_min, self.v_min])
        upper_bound = np.array([self.h_max, self.s_max, self.v_max])
        
        # Maskeyi oluştur
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        return mask
    
    def apply_rgb_filter(self, frame):
        """RGB renk filtresini uygula"""
        # RGB aralığını tanımla (OpenCV BGR formatını kullanır)
        lower_bound = np.array([self.b_min, self.g_min, self.r_min])  # BGR sırası
        upper_bound = np.array([self.b_max, self.g_max, self.r_max])  # BGR sırası
        
        # Maskeyi oluştur
        mask = cv2.inRange(frame, lower_bound, upper_bound)
        
        return mask
    
    def apply_color_filter(self, frame):
        """Seçilen moda göre renk filtresini uygula"""
        if self.use_hsv:
            mask = self.apply_hsv_filter(frame)
        else:
            mask = self.apply_rgb_filter(frame)
        
        # Morfolojik işlemler ile gürültüyü azalt
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Maskeyi orijinal görüntüye uygula
        filtered = cv2.bitwise_and(frame, frame, mask=mask)
        
        return filtered, mask
    
    def add_info_overlay(self, frame):
        """Görüntüye bilgi metni ekle"""
        if self.use_hsv:
            info_text = f"HSV: H({self.h_min}-{self.h_max}) S({self.s_min}-{self.s_max}) V({self.v_min}-{self.v_max})"
            mode_text = "Mod: HSV"
        else:
            info_text = f"RGB: R({self.r_min}-{self.r_max}) G({self.g_min}-{self.g_max}) B({self.b_min}-{self.b_max})"
            mode_text = "Mod: RGB"
        
        # Arka plan için siyah dikdörtgen
        cv2.rectangle(frame, (5, 5), (640, 65), (0, 0, 0), -1)
        
        # Mod bilgisi
        cv2.putText(frame, mode_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Değer bilgisi
        cv2.putText(frame, info_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def run(self):
        """Ana döngü"""
        print("Renk Filtresi Başlatıldı!")
        print("- Mode trackbar'ı ile HSV/RGB arasında geçiş yapın")
        print("- Preset trackbar'ları ile hızlı renk seçimi yapın")
        print("- Değer trackbar'ları ile hassas ayar yapın")
        print("- Başlangıçta mavi renk filtresi aktif")
        print("- Çıkmak için ESC tuşuna basın")
        
        while True:
            if self.use_camera:
                ret, frame = self.cap.read()
                if not ret:
                    print("Kameradan görüntü alınamadı!")
                    break
            else:
                frame = self.create_test_image()
            
            # Görüntüyü yeniden boyutlandır
            frame = cv2.resize(frame, (640, 480))
            
            # Renk filtresini uygula
            filtered_frame, mask = self.apply_color_filter(frame)
            
            # Bilgi overlay'ini ekle
            frame_with_info = self.add_info_overlay(frame.copy())
            filtered_with_info = self.add_info_overlay(filtered_frame.copy())
            
            # Görüntüleri göster
            cv2.imshow(self.window_name, frame_with_info)
            cv2.imshow(self.filtered_window, filtered_with_info)
            cv2.imshow('Maske', mask)
            
            # ESC tuşu ile çıkış
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
        
        # Temizlik
        if self.use_camera:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        color_filter = ColorFilter()
        color_filter.run()
    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından durduruldu.")
    except Exception as e:
        print(f"Hata oluştu: {e}")
        cv2.destroyAllWindows()