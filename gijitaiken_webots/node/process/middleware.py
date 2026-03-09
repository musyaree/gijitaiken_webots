import time

class Middleware:
    """
    Logika Middleware Tachimawari (Sederhana).
    Mengatur 'Siapa boleh menggerakkan Apa' berdasarkan Control Type.
    """
    # Enum Control Type
    DEFAULT = 0
    FOR_WALKING = 1
    FOR_HEAD = 2
    FOR_ACTION = 3
    FORCE = 4

    def __init__(self):
        self.HEAD_IDS = [19, 20]
        self.BODY_IDS = [
            1, 2, 3, 4, 5, 6,             # Arms
            21, 22, 23, 24,               # Grippers & Shoulder Yaw
            7, 8, 9, 10, 11, 12,          # Hips
            13, 14, 15, 16, 17, 18        # Knees & Ankles
        ]

        self.control_rule = self.DEFAULT
        
        # State aktif untuk setiap mode
        # Default: Walking & Action pegang Body, Head pegang Head.
        self.active_ids = {
            self.FOR_WALKING: list(self.BODY_IDS),
            self.FOR_HEAD: list(self.HEAD_IDS),
            self.FOR_ACTION: list(self.BODY_IDS) + list(self.HEAD_IDS)
        }

        # Timestamp untuk prioritas (timeout logic)
        self.last_access = {
            self.FOR_WALKING: 0.0,
            self.FOR_HEAD: 0.0,
            self.FOR_ACTION: 0.0
        }
        self.TIME_LIMIT = 0.5  # 0.5 detik

    def set_rules(self, control_type, ids):
        """
        Dipanggil saat menerima pesan dari 'joint/control_joints'.
        msg.control_type -> int8
        msg.ids -> uint8[] (list of int)
        """
        self.control_rule = control_type
        
        # Jika ids dikirim (tidak kosong), update daftar ID untuk mode tersebut
        if ids:
            self.active_ids[control_type] = list(ids)

    def validate(self, control_type):
        """
        Cek apakah control_type ini diizinkan berjalan saat ini.
        Logic: ACTION memiliki prioritas lebih tinggi dari WALKING selama durasi timeout.
        """
        if control_type == self.FORCE:
            return True

        current_time = time.time()
        
        # Update kapan terakhir kali mode ini meminta akses
        if control_type in self.last_access:
            self.last_access[control_type] = current_time

        # Jika ACTION baru saja aktif (< 0.5 detik lalu), WALKING ditahan dulu.
        if control_type == self.FOR_WALKING:
            if (current_time - self.last_access[self.FOR_ACTION]) < self.TIME_LIMIT:
                return False
        
        return True

    def filter_joints(self, control_type, joints_msg):
        """
        Hanya loloskan joint yang ID-nya ada di daftar izin mode tersebut.
        """
        if control_type == self.FORCE:
            return joints_msg

        # Ambil daftar ID yang boleh digerakkan oleh mode ini
        allowed_ids = self.active_ids.get(control_type, [])
        
        filtered_list = []
        for joint in joints_msg:
            if joint.id in allowed_ids:
                filtered_list.append(joint)
        
        return filtered_list