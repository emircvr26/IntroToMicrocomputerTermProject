import customtkinter as ctk
from tkinter import messagebox
import threading
import time
from api import AirConditionerSystemConnection, CurtainControlSystemConnection

ctk.set_appearance_mode("Light")
ctk.set_default_color_theme("blue") 

COLOR_BG = "#F5F5F7"           
COLOR_CARD = "#FFFFFF"         
COLOR_TEXT_MAIN = "#1D1D1F"    
COLOR_TEXT_BOLD = "#000000"    
COLOR_ACCENT = "#007AFF"       
COLOR_GREEN_TEXT = "#34C759"   
COLOR_BORDER = "#E5E5EA"       
COLOR_RED_BOX = "#D32F2F"      

class HomeControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Home Control Center")
        self.geometry("900x650") 
        self.configure(fg_color=COLOR_BG)
        self.resizable(False, False)

        self.ac_api = AirConditionerSystemConnection()
        self.curtain_api = CurtainControlSystemConnection()
        self.is_connected = False

        
        self.grid_rowconfigure(0, weight=0) 
        self.grid_rowconfigure(1, weight=1) 
        self.grid_columnconfigure(0, weight=1)

        self.create_top_bar()

        self.main_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.main_frame.grid(row=1, column=0, sticky="nsew", padx=20, pady=20)
        
        self.main_frame.grid_columnconfigure(0, weight=1) 
        self.main_frame.grid_columnconfigure(1, weight=1) 
        self.main_frame.grid_rowconfigure(0, weight=1) 
        self.main_frame.grid_rowconfigure(1, weight=1) 

        self.create_ac_card()
        self.create_curtain_card()
        self.create_log_panel()

        self.running = True
        self.update_thread = threading.Thread(target=self.data_loop, daemon=True)
        self.update_thread.start()

    def create_top_bar(self):
        header = ctk.CTkFrame(self, fg_color=COLOR_CARD, corner_radius=0, height=60)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_propagate(False)

        ctk.CTkLabel(header, text="CONTROL PANEL", font=("Arial", 16, "bold"), text_color=COLOR_TEXT_MAIN).place(relx=0.5, rely=0.5, anchor="center")

        conn_frame = ctk.CTkFrame(header, fg_color="transparent")
        conn_frame.place(relx=0.97, rely=0.5, anchor="e")

        ctk.CTkLabel(conn_frame, text="COM", font=("Arial", 12, "bold"), text_color=COLOR_TEXT_MAIN).pack(side="left", padx=(0, 5))

        # Port 1 
        self.entry_p1 = ctk.CTkEntry(conn_frame, placeholder_text="1", width=40, font=("Arial", 12, "bold"), justify="center",
                                     fg_color=COLOR_RED_BOX, text_color="white", border_width=0)
        self.entry_p1.pack(side="left", padx=2)
        self.entry_p1.insert(0, "1")

        # Port 2 
        self.entry_p2 = ctk.CTkEntry(conn_frame, placeholder_text="2", width=40, font=("Arial", 12, "bold"), justify="center",
                                     fg_color=COLOR_RED_BOX, text_color="white", border_width=0)
        self.entry_p2.pack(side="left", padx=2)
        self.entry_p2.insert(0, "2")

        self.btn_connect = ctk.CTkButton(conn_frame, text="CONNECT", width=90, height=28, 
                                         font=("Arial", 11, "bold"), command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=(10, 0))

    def create_ac_card(self):
        self.card_ac = ctk.CTkFrame(self.main_frame, fg_color=COLOR_CARD, corner_radius=10, border_color=COLOR_BORDER, border_width=1)
        self.card_ac.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(0, 10))
        
        head = ctk.CTkFrame(self.card_ac, fg_color="transparent")
        head.pack(fill="x", padx=15, pady=10)
        ctk.CTkLabel(head, text="AIR CONDITIONING SYSTEM", font=("Arial", 12, "bold"), text_color=COLOR_ACCENT).pack(side="left")

        self.lbl_ac_temp = ctk.CTkLabel(self.card_ac, text="--.--°C", font=("Arial", 48, "bold"), text_color=COLOR_TEXT_MAIN)
        self.lbl_ac_temp.pack(pady=(5, 0))

        self.lbl_fan = ctk.CTkLabel(self.card_ac, text="Fan Speed: -- RPS", font=("Arial", 16, "bold"), text_color=COLOR_TEXT_BOLD)
        self.lbl_fan.pack(pady=(0, 15))

        ctrl = ctk.CTkFrame(self.card_ac, fg_color="transparent")
        ctrl.pack(fill="x", padx=30, pady=5)
        
        self.slider_ac = ctk.CTkEntry(ctrl, placeholder_text="25.0", width=80, font=("Arial", 14), justify="center")
        self.slider_ac.pack(side="left", padx=(0, 10))
        self.slider_ac.insert(0, "25.0")
        
        ctk.CTkButton(ctrl, text="SET", width=100, height=30, font=("Arial", 11, "bold"), command=self.cmd_set_temp).pack(side="left")

        self.lbl_target = ctk.CTkLabel(self.card_ac, text="Target: -- °C", font=("Arial", 20, "bold"), text_color=COLOR_GREEN_TEXT)
        self.lbl_target.pack(pady=(10, 10))

    def create_curtain_card(self):
        self.card_curtain = ctk.CTkFrame(self.main_frame, fg_color=COLOR_CARD, corner_radius=10, border_color=COLOR_BORDER, border_width=1)
        self.card_curtain.grid(row=1, column=0, sticky="nsew", padx=(0, 10), pady=(0, 0))

        head = ctk.CTkFrame(self.card_curtain, fg_color="transparent")
        head.pack(fill="x", padx=15, pady=10)
        ctk.CTkLabel(head, text="CURTAIN CONTROL SYSTEM", font=("Arial", 12, "bold"), text_color=COLOR_ACCENT).pack(side="left")

        grid = ctk.CTkFrame(self.card_curtain, fg_color="transparent")
        grid.pack(fill="x", padx=15)
        grid.grid_columnconfigure((0,1), weight=1)

        self.val_light = self.mini_info(grid, 0, 0, "Light")
        
        self.val_curtain = self.mini_info(grid, 0, 1, "Curtain Status")
        
        self.val_pressure = self.mini_info(grid, 1, 0, "Pressure")
        
        self.val_outdoor = self.mini_info(grid, 1, 1, "Outdoor Temp")

        ctk.CTkFrame(self.card_curtain, height=1, fg_color="#EEEEEE").pack(fill="x", padx=20, pady=15)

        ctrl_frame = ctk.CTkFrame(self.card_curtain, fg_color="transparent")
        ctrl_frame.pack(fill="x", padx=30, pady=5)

        self.entry_curtain = ctk.CTkEntry(ctrl_frame, placeholder_text="%", width=80, font=("Arial", 14), justify="center")
        self.entry_curtain.pack(side="left", padx=(0, 10))
        self.entry_curtain.insert(0, "0") 

        ctk.CTkButton(ctrl_frame, text="SET", width=100, height=30, font=("Arial", 11, "bold"), command=self.cmd_set_curtain).pack(side="left")

        ctk.CTkLabel(self.card_curtain, text="(0 = Open, 100 = Closed)", font=("Arial", 10), text_color="gray").pack(pady=5)

    def create_log_panel(self):
        log_frame = ctk.CTkFrame(self.main_frame, fg_color=COLOR_CARD, corner_radius=10, border_color=COLOR_BORDER, border_width=1)
        log_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")

        ctk.CTkLabel(log_frame, text="SYSTEM LOG", font=("Arial", 10, "bold"), text_color="gray").pack(anchor="w", padx=10, pady=(10,5))
        
        self.terminal = ctk.CTkTextbox(log_frame, font=("Consolas", 10), text_color="#333", fg_color="#FAFAFA")
        self.terminal.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.log("System ready.")

    def mini_info(self, parent, r, c, title):
        f = ctk.CTkFrame(parent, fg_color="transparent", corner_radius=5)
        f.grid(row=r, column=c, padx=3, pady=3, sticky="nsew")
        
        ctk.CTkLabel(f, text=title, font=("Arial", 9), text_color="gray").pack(pady=(3,0))
        l = ctk.CTkLabel(f, text="--", font=("Arial", 12, "bold"), text_color=COLOR_TEXT_MAIN)
        l.pack(pady=(0,3))
        return l

    def log(self, msg):
        self.terminal.insert("end", f"{msg}\n")
        self.terminal.see("end")

    def toggle_connection(self):
        if not self.is_connected:
            p1 = self.entry_p1.get()
            p2 = self.entry_p2.get()
            if not p1 or not p2: return

            self.ac_api.setComPort(int(p1))
            self.curtain_api.setComPort(int(p2))

            if self.ac_api.open() and self.curtain_api.open():
                self.is_connected = True
                self.btn_connect.configure(text="DISCONNECT", fg_color="#FF3B30", hover_color="#D32F2F")
                self.log(f"Connected (P:{p1}, P:{p2})")
            else:
                self.log("Error: Port Open Failed")
        else:
            self.ac_api.close()
            self.curtain_api.close()
            self.is_connected = False
            self.btn_connect.configure(text="CONNECT", fg_color=COLOR_ACCENT, hover_color="#005BB5")
            self.log("Connection closed.")

    def cmd_set_temp(self):
        if self.is_connected:
            try:
                val = float(self.slider_ac.get())
                self.ac_api.setDesiredTemp(val)
                self.log(f"AC Command: {val}°C")
            except: 
                self.log("Error: Invalid Number")

    def cmd_set_curtain(self):
        if self.is_connected:
            try:
                val = float(self.entry_curtain.get()) 
                if val < 0: val = 0
                if val > 100: val = 100
                
                self.curtain_api.setCurtainStatus(val)
                self.log(f"Curtain Cmd: %{int(val)}")
            except:
                self.log("Error: Enter number for curtain")

    def data_loop(self):
        while self.running:
            if self.is_connected:
                try:
                    self.ac_api.update()
                    self.lbl_ac_temp.configure(text=f"{self.ac_api.ambientTemperature:.1f}°C")
                    self.lbl_fan.configure(text=f"Fan Speed: {self.ac_api.fanSpeed} RPS")
                    self.lbl_target.configure(text=f"Target: {self.ac_api.desiredTemperature:.1f} °C")

                    self.curtain_api.update()
                    self.val_outdoor.configure(text=f"{self.curtain_api.outdoorTemperature:.1f}°")
                    self.val_pressure.configure(text=f"{int(self.curtain_api.outdoorPressure)} hPa")
                    self.val_light.configure(text=f"{int(self.curtain_api.lightIntensity)} Lx")
                    self.val_curtain.configure(text=f"%{int(self.curtain_api.curtainStatus)}")
                except: pass
            time.sleep(1)

if __name__ == "__main__":
    app = HomeControlApp()
    app.mainloop()