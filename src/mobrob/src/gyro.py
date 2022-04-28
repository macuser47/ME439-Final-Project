class Gyro:
    def __init__(self, device_id=0):
        self.dev_id = device_id
        with open(f'/sys/bus/iio/devices/iio:device{self.dev_id}/in_anglvel_scale', 'r') as f:
            self.angvel_scale = float(f.read())
        self.angvel_z_file = open(f'/sys/bus/iio/devices/iio:device{self.dev_id}/in_anglvel_z_raw', 'r')
        self.z_bias = 52.95
        
    def angular_velocity(self):
        ''' Return the angular velocity about the z axis in rad/s '''
        self.angvel_z_file.seek(0)
        return (int(self.angvel_z_file.readline()) + self.z_bias) * self.angvel_scale

if __name__ == '__main__':
    import time
    g = Gyro()
    angle = 0
    last = time.time()
    while True:
        print(angle * (180 / 3.14159265358979323))
        angvel = g.angular_velocity()
        now = time.time()
        dt = now - last
        angle += angvel * dt

        last = now
