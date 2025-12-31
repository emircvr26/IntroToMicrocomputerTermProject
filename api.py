import serial
import time
import struct

class HomeAutomationSystemConnection:
    def __init__(self):
        self.comPort = 0
        self.baudRate = 9600
        self.serial_connection = None

    def open(self):
        try:
            port_name = f'COM{self.comPort}' 
            self.serial_connection = serial.Serial(
                port=port_name,
                baudrate=self.baudRate,
                timeout=2 
            )
            print(f"Bağlantı Başarılı: {port_name}")
            return True
        except Exception as e:
            print(f"HATA: Bağlantı açılamadı ({e})")
            return False

    def close(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Bağlantı kapatıldı.")
            return True
        return False

    def setComPort(self, port):
        self.comPort = port

    def setBaudRate(self, rate):
        self.baudRate = rate

    def _send_byte(self, byte_val):
        """Yardımcı fonksiyon: Tek bir byte gönderir"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(bytes([byte_val]))
            time.sleep(0.05) 

    def _read_byte(self):
        """Yardımcı fonksiyon: Tek bir byte okur"""
        if self.serial_connection and self.serial_connection.is_open:
            data = self.serial_connection.read(1)
            if data:
                return ord(data)
        return 0

# Board 1
class AirConditionerSystemConnection(HomeAutomationSystemConnection):
    def __init__(self):
        super().__init__()
        self.desiredTemperature = 0.0
        self.ambientTemperature = 0.0
        self.fanSpeed = 0

    def update(self):
        self.desiredTemperature = self.getDesiredTemp()
        self.ambientTemperature = self.getAmbientTemp()
        self.fanSpeed = self.getFanSpeed()

    def setDesiredTemp(self, temp):
        
        integral_part = int(temp)
        fractional_part = int((temp - integral_part) * 10) 

        integral_part = integral_part & 0x3F 
        fractional_part = fractional_part & 0x3F

        cmd_low = 0b10000000 | fractional_part
        self._send_byte(cmd_low)

        cmd_high = 0b11000000 | integral_part
        self._send_byte(cmd_high)
        
        print(f"Sıcaklık ayarlandı: {temp} (Int: {integral_part}, Frac: {fractional_part})")
        return True

    def getDesiredTemp(self):
        
        self._send_byte(0b00000001)
        frac = self._read_byte()
        
        self._send_byte(0b00000010)
        integ = self._read_byte()
        
        self.desiredTemperature = float(integ) + (float(frac) / 10.0)
        return self.desiredTemperature

    def getAmbientTemp(self):
        
        self._send_byte(0b00000011)
        frac = self._read_byte()
        
        self._send_byte(0b00000100)
        integ = self._read_byte()
        
        self.ambientTemperature = float(integ) + (float(frac) / 10.0)
        return self.ambientTemperature

    def getFanSpeed(self):
        self._send_byte(0b00000101)
        self.fanSpeed = self._read_byte()
        return self.fanSpeed

# Board 2
class CurtainControlSystemConnection(HomeAutomationSystemConnection):
    def __init__(self):
        super().__init__()
        self.curtainStatus = 0.0
        self.outdoorTemperature = 0.0
        self.outdoorPressure = 0.0
        self.lightIntensity = 0.0

    def update(self):
        
        self._send_byte(0b00000001)
        c_low = self._read_byte()
        self._send_byte(0b00000010)
        c_high = self._read_byte()
        self.curtainStatus = float(c_high) + (float(c_low) / 10.0)

        self._send_byte(0b00000011)
        t_low = self._read_byte()
        self._send_byte(0b00000100)
        t_high = self._read_byte()
        self.outdoorTemperature = float(t_high) + (float(t_low) / 10.0)

        self._send_byte(0b00000101)
        p_low = self._read_byte()
        self._send_byte(0b00000110)
        p_high = self._read_byte()
        self.outdoorPressure = float(p_high) + (float(p_low) / 10.0) 

        self._send_byte(0b00000111)
        l_low = self._read_byte()
        self._send_byte(0b00001000)
        l_high = self._read_byte()
        self.lightIntensity = float(l_high) + (float(l_low) / 10.0)

    def setCurtainStatus(self, status):
        val = int(status)
        if val > 100: val = 100
        if val < 0: val = 0

       
        cmd = 0x80 | val
        
        self._send_byte(cmd)
        print(f"Perde ayarlandı: %{val}")
        return True
    
    def getOutdoorTemp(self):
        return self.outdoorTemperature
    
    def getOutdoorPress(self):
        return self.outdoorPressure
        
    def getLightIntensity(self):
        return self.lightIntensity