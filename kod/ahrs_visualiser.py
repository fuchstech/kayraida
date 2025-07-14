import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
from pymavlink import mavutil
from matplotlib.animation import FuncAnimation
import queue

class AircraftVisualizer:
    def __init__(self, mavlink_connection='COM7', baud=115200):
        # MAVLink bağlantısı
        self.mavlink_connection = mavlink_connection
        self.baud = baud
        self.master = None
        
        # Orientasyon verileri için queue
        self.data_queue = queue.Queue()
        
        # Mevcut orientasyon
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 3D plot setup
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Uçak modeli için temel noktalar
        self.create_aircraft_model()
        
        # Threading için flag
        self.running = True
        
    def create_aircraft_model(self):
        """Basit bir uçak modeli oluştur"""
        # Uçak gövdesi (fuselage)
        self.fuselage_x = np.array([-2, 2])
        self.fuselage_y = np.array([0, 0])
        self.fuselage_z = np.array([0, 0])
        
        # Kanatlar (wings)
        self.wing_x = np.array([-0.5, 0.5])
        self.wing_y = np.array([-3, 3])
        self.wing_z = np.array([0, 0])
        
        # Dikey kuyruk (vertical tail)
        self.vtail_x = np.array([-1.5, -1.5])
        self.vtail_y = np.array([0, 0])
        self.vtail_z = np.array([0, 1.5])
        
        # Yatay kuyruk (horizontal tail)
        self.htail_x = np.array([-1.5, -1.5])
        self.htail_y = np.array([-1, 1])
        self.htail_z = np.array([0, 0])
        
    def rotation_matrix(self, roll, pitch, yaw):
        """Euler açılarından rotasyon matrisi oluştur"""
        # Roll (x ekseni etrafında)
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        # Pitch (y ekseni etrafında)
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        # Yaw (z ekseni etrafında)
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        # Toplam rotasyon matrisi
        R = R_z @ R_y @ R_x
        return R
        
    def transform_aircraft(self):
        """Uçak modelini mevcut orientasyona göre dönüştür"""
        R = self.rotation_matrix(self.roll, self.pitch, self.yaw)
        
        # Tüm uçak parçalarını dönüştür
        parts = [
            np.array([self.fuselage_x, self.fuselage_y, self.fuselage_z]),
            np.array([self.wing_x, self.wing_y, self.wing_z]),
            np.array([self.vtail_x, self.vtail_y, self.vtail_z]),
            np.array([self.htail_x, self.htail_y, self.htail_z])
        ]
        
        transformed_parts = []
        for part in parts:
            transformed = R @ part
            transformed_parts.append(transformed)
            
        return transformed_parts
        
    def mavlink_reader(self):
        """MAVLink verilerini okuyan thread fonksiyonu"""
        try:
            print(f"Bağlantı kuruluyor: {self.mavlink_connection} @ {self.baud}")
            self.master = mavutil.mavlink_connection(self.mavlink_connection, baud=self.baud)
            print("✓ MAVLink bağlantısı başarılı!")
            
            message_count = 0
            last_print_time = time.time()
            
            while self.running:
                msg = self.master.recv_match(timeout=1)
                if not msg:
                    continue
                    
                if msg.get_type() == 'AHRS2':
                    # Yeni veri geldiğinde queue'ya ekle
                    self.data_queue.put((msg.roll, msg.pitch, msg.yaw))
                    message_count += 1
                    
                    # Her 5 saniyede bir mesaj sayısını göster
                    current_time = time.time()
                    if current_time - last_print_time >= 5.0:
                        print(f"✓ AHRS2 mesajları alınıyor... (Son 5s: {message_count} mesaj)")
                        message_count = 0
                        last_print_time = current_time
                    
        except Exception as e:
            print(f"✗ MAVLink bağlantı hatası: {e}")
            print("Lütfen kontrol edin:")
            print("- Cihaz bağlı mı?")
            print("- COM port doğru mu?") 
            print("- Baud rate doğru mu?")
            print("- Başka program portu kullanıyor mu?")
            
    def update_plot(self, frame):
        """Animasyon için plot güncelleme fonksiyonu"""
        # Queue'dan yeni veri varsa al
        data_received = False
        try:
            while not self.data_queue.empty():
                self.roll, self.pitch, self.yaw = self.data_queue.get_nowait()
                data_received = True
        except queue.Empty:
            pass
            
        # Plot'u temizle
        self.ax.clear()
        
        # Dönüştürülmüş uçak modelini al
        transformed_parts = self.transform_aircraft()
        
        # Uçak parçalarını çiz
        colors = ['red', 'blue', 'green', 'orange']
        labels = ['Gövde', 'Kanatlar', 'Dikey Kuyruk', 'Yatay Kuyruk']
        
        for i, part in enumerate(transformed_parts):
            self.ax.plot(part[0], part[1], part[2], 
                        color=colors[i], linewidth=3, label=labels[i])
        
        # Koordinat sistemini çiz
        origin = np.array([0, 0, 0])
        self.ax.quiver(0, 0, 0, 2, 0, 0, color='red', alpha=0.6, arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 2, 0, color='green', alpha=0.6, arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 0, 2, color='blue', alpha=0.6, arrow_length_ratio=0.1)
        
        # Eksen ayarları
        self.ax.set_xlim([-4, 4])
        self.ax.set_ylim([-4, 4])
        self.ax.set_zlim([-2, 3])
        
        self.ax.set_xlabel('X (İleri)')
        self.ax.set_ylabel('Y (Sağ)')
        self.ax.set_zlabel('Z (Yukarı)')
        
        # Orientasyon bilgilerini göster
        status_text = (f'Roll: {np.degrees(self.roll):.2f}°\n'
                      f'Pitch: {np.degrees(self.pitch):.2f}°\n'
                      f'Yaw: {np.degrees(self.yaw):.2f}°')
        
        # Veri alma durumunu göster
        if hasattr(self, 'master') and self.master:
            status_text += '\n✓ MAVLink Bağlı'
        else:
            status_text += '\n✗ MAVLink Bekleniyor...'
            
        self.ax.text2D(0.02, 0.98, status_text,
                      transform=self.ax.transAxes, 
                      verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.ax.legend()
        self.ax.set_title('Gerçek Zamanlı Uçak Orientasyonu - MAVLink AHRS2')
        
    def start_visualization(self):
        """Görselleştirmeyi başlat"""
        print("MAVLink okuma thread'i başlatılıyor...")
        
        # MAVLink okuma thread'ini başlat
        mavlink_thread = threading.Thread(target=self.mavlink_reader)
        mavlink_thread.daemon = True
        mavlink_thread.start()
        
        # Thread'in başlaması için kısa bir bekleme
        time.sleep(1)
        
        print("3D görselleştirme başlatılıyor...")
        
        # Animasyonu başlat
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        # Pencereyi göster
        plt.show()
        
    def stop(self):
        """Görselleştirmeyi durdur"""
        self.running = False
        if self.master:
            self.master.close()

def main():
    """Ana fonksiyon"""
    print("Uçak Orientasyon Görselleştirici - MAVLink Bağlantısı")
    print("=" * 50)
    
    # COM port ve baud rate ayarları
    com_port = input("COM port (varsayılan COM7): ").strip() or "COM7"
    baud_input = input("Baud rate (varsayılan 115200): ").strip()
    baud_rate = int(baud_input) if baud_input else 115200
    
    try:
        print(f"\nMAVLink bağlantısı kuruluyor: {com_port} @ {baud_rate}")
        visualizer = AircraftVisualizer(com_port, baud_rate)
        
        print("Görselleştirme başlatılıyor...")
        print("AHRS2 mesajları bekleniyor...")
        print("Pencereyi kapatarak programdan çıkabilirsiniz.")
        
        visualizer.start_visualization()
        
    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından durduruldu.")
    except ValueError:
        print("Geçersiz baud rate değeri!")
    except Exception as e:
        print(f"Hata: {e}")
        print("Lütfen COM port ve bağlantıyı kontrol edin.")
    finally:
        if 'visualizer' in locals():
            visualizer.stop()

if __name__ == "__main__":
    main()