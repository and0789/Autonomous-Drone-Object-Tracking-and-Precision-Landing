import cv2
import config  # Import konfigurasi dari config.py

class ObjectTracker:
    """
    Kelas untuk melakukan pelacakan objek (object tracking) menggunakan OpenCV.
    Mendukung 8 jenis tracker: BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN, MOSSE, dan CSRT.
    """

    def __init__(self):
        """
        Inisialisasi ObjectTracker dengan tracker kosong.
        Mode tracking diambil dari config.py.
        """
        self.tracker = None
        self.mode = config.TRACKING_MODE  # Mengambil mode tracking dari config.py

    def init_tracker(self, frame, bbox):
        """
        Inisialisasi tracker dengan frame dan bounding box baru.

        :param frame: Frame gambar saat ini.
        :param bbox: Bounding box dalam format (x, y, width, height).
        """
        # Reset tracker jika sudah ada
        if self.tracker is not None:
            self.reset_tracker()

        mode_upper = self.mode.upper()  # Konversi mode ke huruf kapital agar perbandingan tidak case-sensitive

        # Pemilihan tracker berdasarkan mode dari config.py
        if mode_upper == "BOOSTING":
            try:
                self.tracker = cv2.legacy.TrackerBoosting_create()
            except AttributeError:
                self.tracker = cv2.TrackerBoosting_create()
        elif mode_upper == "MIL":
            try:
                self.tracker = cv2.legacy.TrackerMIL_create()
            except AttributeError:
                self.tracker = cv2.TrackerMIL_create()
        elif mode_upper == "KCF":
            try:
                self.tracker = cv2.legacy.TrackerKCF_create()
            except AttributeError:
                self.tracker = cv2.TrackerKCF_create()
        elif mode_upper == "TLD":
            try:
                self.tracker = cv2.legacy.TrackerTLD_create()
            except AttributeError:
                self.tracker = cv2.TrackerTLD_create()
        elif mode_upper == "MEDIANFLOW":
            try:
                self.tracker = cv2.legacy.TrackerMedianFlow_create()
            except AttributeError:
                self.tracker = cv2.TrackerMedianFlow_create()
        elif mode_upper == "GOTURN":
            try:
                self.tracker = cv2.legacy.TrackerGOTURN_create()
            except AttributeError:
                self.tracker = cv2.TrackerGOTURN_create()
        elif mode_upper == "MOSSE":
            try:
                self.tracker = cv2.legacy.TrackerMOSSE_create()
            except AttributeError:
                self.tracker = cv2.TrackerMOSSE_create()
        elif mode_upper == "CSRT":
            try:
                self.tracker = cv2.legacy.TrackerCSRT_create()
            except AttributeError:
                self.tracker = cv2.TrackerCSRT_create()
        else:
            raise ValueError(f"Mode tracking '{self.mode}' tidak dikenali. Gunakan salah satu dari: "
                             "BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN, MOSSE, CSRT.")

        # Inisialisasi tracker dengan frame dan bounding box yang diberikan
        self.tracker.init(frame, bbox)
        print(f"Tracker {self.mode} diinisialisasi dengan bbox: {bbox}")

    def update_tracker(self, frame):
        """
        Update tracker dengan frame terbaru.

        :param frame: Frame gambar saat ini.
        :return: Tuple (success, bbox) di mana 'success' adalah boolean dan 'bbox'
                 adalah bounding box yang terdeteksi (jika tracking berhasil).
        """
        if self.tracker is None:
            return False, None

        success, bbox = self.tracker.update(frame)
        return success, bbox

    def reset_tracker(self):
        """
        Reset tracker dengan mengatur tracker menjadi None.
        """
        self.tracker = None
        print("Tracker di-reset.")

# Contoh penggunaan:
# tracker = ObjectTracker()  # Mode akan diambil dari config.py
# tracker.init_tracker(frame, bbox)
