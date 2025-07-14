import threading
import time

def gorev(isim, bekleme_suresi):
    print(f"{isim} başladı")
    time.sleep(bekleme_suresi)
    print(f"{isim} bitti")

def main():
    print("=== SİRALI ÇALIŞMA (Threading Yok) ===")
    baslangic = time.time()
    
    gorev("İş-1", 2)
    gorev("İş-2", 2)
    gorev("İş-3", 2)
    
    bitis = time.time()
    print(f"Toplam süre: {bitis - baslangic:.1f} saniye\n")

    print("=== PARALEL ÇALIŞMA (Threading Var) ===")
    baslangic = time.time()
    
    t1 = threading.Thread(target=gorev, args=("Thread-1", 2))
    t2 = threading.Thread(target=gorev, args=("Thread-2", 2))
    t3 = threading.Thread(target=gorev, args=("Thread-3", 2))
    
    t1.start()
    t2.start()
    t3.start()
    
    t1.join()
    t2.join()
    t3.join()
    
    bitis = time.time()
    print(f"Toplam süre: {bitis - baslangic:.1f} saniye")

if __name__ == "__main__":
    main()