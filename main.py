import wx
import serial
import threading
import time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
import numpy as np


class DCONController:
    def __init__(self):
        self.port = "COM3"
        self.baudrate = 57600
        self.data_bits = 8
        self.stop_bits = 1
        self.parity = 'N'
        self.timeout = 0.2
        self.ser = None
        self.lock = threading.Lock()

        # Адреса устройств
        self.adrDO = 1
        self.adrAO = 2
        self.valve1 = 1
        self.valve2 = 4
        self.valve3 = 3
        self.valve4 = 2
        self.startPUMP = 5
        self.plusPUMP = 6
        self.minusPUMP = 7
        self.pressureChanel = 5

        # Параметры
        self.minMA = 3.86
        self.maxMA = 19.368
        self.minP = 0
        self.maxP = 6

        # Переменные состояния
        self.pressurePOSLE = 0.0
        self.stop_igra = False
        self.plus_counter = 0
        self.minus_counter = 0
        self.fixed_open = 0.0
        self.fixed_close = 0.0
        self.plus_dim_press = [0.0] * 41

        # Тайминги
        self.plus_minus_time = 100
        self.plus_step_time = 1000
        self.minus_step_time = 1000
        self.plus_step = 80
        self.minus_step = 40

    def connect(self):
        """Подключение к COM-порту"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.data_bits,
                stopbits=self.stop_bits,
                parity=self.parity,
                timeout=self.timeout
            )
            return True
        except Exception as e:
            print(f"Ошибка подключения: {e}")
            return False

    def disconnect(self):
        """Отключение от COM-порта"""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_command(self, command):
        """Отправка команды устройству"""
        with self.lock:
            try:
                if not self.ser or not self.ser.is_open:
                    if not self.connect():
                        return False

                self.ser.write(command.encode())
                time.sleep(0.1)
                return True
            except Exception as e:
                print(f"Ошибка отправки команды: {e}")
                return False

    def send_705X_XX(self, module_id, address, channel, value):
        """Отправка команды модулю 705X"""
        cmd = f"{module_id:04X}{address:02X}{channel:02X}{value:02X}\r"
        return self.send_command(cmd)

    def read_7019_2(self):
        """Чтение данных с модуля 7019"""
        with self.lock:
            try:
                if not self.ser or not self.ser.is_open:
                    if not self.connect():
                        return None

                # Команда чтения
                cmd = f"7017{self.adrAO:02X}00\r"
                self.ser.write(cmd.encode())
                time.sleep(0.1)

                # Чтение ответа
                response = self.ser.readline().decode().strip()
                if response:
                    # Парсинг ответа (упрощенно)
                    values = response.split()
                    if len(values) > self.pressureChanel:
                        pressure_raw = float(values[self.pressureChanel])
                        # Нормирование значения
                        pressure_norm = abs(pressure_raw - self.minMA) * (self.maxP - self.minP) / (
                                    self.maxMA - self.minMA)
                        self.pressurePOSLE = round(pressure_norm, 2)
                        return self.pressurePOSLE

                return None
            except Exception as e:
                print(f"Ошибка чтения: {e}")
                return None


class MainFrame(wx.Frame):
    def __init__(self):
        super().__init__(None, title="Система управления насосами и клапанами", size=(1000, 700))
        self.controller = DCONController()
        self.timer = wx.Timer(self)
        self.test_timer = None
        self.is_test_running = False

        self.init_ui()
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.Bind(wx.EVT_CLOSE, self.on_close)

        # Запуск таймера для обновления давления
        self.timer.Start(1000)

    def init_ui(self):
        """Инициализация интерфейса"""
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Панель давления
        pressure_sizer = wx.BoxSizer(wx.HORIZONTAL)
        pressure_sizer.Add(wx.StaticText(panel, label="Давление: "), 0, wx.ALIGN_CENTER | wx.ALL, 5)
        self.txt_pressure = wx.TextCtrl(panel, value="0.0", style=wx.TE_READONLY)
        pressure_sizer.Add(self.txt_pressure, 1, wx.EXPAND | wx.ALL, 5)
        self.btn_read_pressure = wx.Button(panel, label="Обновить")
        self.btn_read_pressure.Bind(wx.EVT_BUTTON, self.on_read_pressure)
        pressure_sizer.Add(self.btn_read_pressure, 0, wx.ALL, 5)

        # Панель управления насосом
        pump_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.btn_pump = wx.Button(panel, label="Пуск насоса")
        self.btn_pump.SetBackgroundColour(wx.Colour(0, 255, 0))  # Зеленый
        self.btn_pump.Bind(wx.EVT_BUTTON, self.on_pump_toggle)
        pump_sizer.Add(self.btn_pump, 1, wx.EXPAND | wx.ALL, 5)

        self.btn_plus = wx.Button(panel, label="+ Частота")
        self.btn_plus.Bind(wx.EVT_BUTTON, self.on_plus_frequency)
        pump_sizer.Add(self.btn_plus, 0, wx.ALL, 5)

        self.btn_minus = wx.Button(panel, label="- Частота")
        self.btn_minus.Bind(wx.EVT_BUTTON, self.on_minus_frequency)
        pump_sizer.Add(self.btn_minus, 0, wx.ALL, 5)

        # Панель управления клапанами
        valves_sizer = wx.GridSizer(2, 2, 5, 5)

        self.btn_to_forward = wx.Button(panel, label="В клапан прямой")
        self.btn_to_forward.SetBackgroundColour(wx.Colour(255, 0, 0))  # Красный
        self.btn_to_forward.Bind(wx.EVT_BUTTON, self.on_to_forward)
        valves_sizer.Add(self.btn_to_forward, 0, wx.EXPAND)

        self.btn_to_revers = wx.Button(panel, label="В клапан обратный")
        self.btn_to_revers.SetBackgroundColour(wx.Colour(255, 0, 0))
        self.btn_to_revers.Bind(wx.EVT_BUTTON, self.on_to_revers)
        valves_sizer.Add(self.btn_to_revers, 0, wx.EXPAND)

        self.btn_from_forward = wx.Button(panel, label="Из клапана прямой")
        self.btn_from_forward.SetBackgroundColour(wx.Colour(255, 0, 0))
        self.btn_from_forward.Bind(wx.EVT_BUTTON, self.on_from_forward)
        valves_sizer.Add(self.btn_from_forward, 0, wx.EXPAND)

        self.btn_from_revers = wx.Button(panel, label="Из клапана обратный")
        self.btn_from_revers.SetBackgroundColour(wx.Colour(255, 0, 0))
        self.btn_from_revers.Bind(wx.EVT_BUTTON, self.on_from_revers)
        valves_sizer.Add(self.btn_from_revers, 0, wx.EXPAND)

        # Панель тестов
        test_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.btn_forward_test = wx.Button(panel, label="Прямые испытания")
        self.btn_forward_test.Bind(wx.EVT_BUTTON, self.on_forward_test)
        test_sizer.Add(self.btn_forward_test, 1, wx.EXPAND | wx.ALL, 5)

        self.btn_revers_test = wx.Button(panel, label="Обратные испытания")
        self.btn_revers_test.Bind(wx.EVT_BUTTON, self.on_revers_test)
        test_sizer.Add(self.btn_revers_test, 1, wx.EXPAND | wx.ALL, 5)

        self.btn_stop = wx.Button(panel, label="СТОП")
        self.btn_stop.Bind(wx.EVT_BUTTON, self.on_stop)
        test_sizer.Add(self.btn_stop, 1, wx.EXPAND | wx.ALL, 5)

        # Текстовое поле для логов
        self.text_memo = wx.TextCtrl(panel, style=wx.TE_MULTILINE | wx.TE_READONLY)

        # График
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.canvas = FigureCanvas(panel, -1, self.fig)
        self.ax.set_title('График давления')
        self.ax.set_xlabel('Шаги')
        self.ax.set_ylabel('Давление (атм)')
        self.line, = self.ax.plot([], [])
        self.ax.set_ylim(0, 6)

        # Компоновка элементов
        main_sizer.Add(pressure_sizer, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(pump_sizer, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(valves_sizer, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(test_sizer, 0, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(self.text_memo, 1, wx.EXPAND | wx.ALL, 5)
        main_sizer.Add(self.canvas, 1, wx.EXPAND | wx.ALL, 5)

        panel.SetSizer(main_sizer)
        self.Centre()

        # Подключение к устройству
        if self.controller.connect():
            self.log_message("Успешно подключено к COM-порту")
        else:
            self.log_message("Ошибка подключения к COM-порту")

    def log_message(self, message):
        """Добавление сообщения в лог"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.text_memo.AppendText(f"[{timestamp}] {message}\n")

    def on_timer(self, event):
        """Обновление давления по таймеру"""
        pressure = self.controller.read_7019_2()
        if pressure is not None:
            self.txt_pressure.SetValue(f"{pressure:.2f}")

    def on_read_pressure(self, event):
        """Ручное обновление давления"""
        pressure = self.controller.read_7019_2()
        if pressure is not None:
            self.txt_pressure.SetValue(f"{pressure:.2f}")
            self.log_message(f"Текущее давление: {pressure:.2f} атм")

    def on_pump_toggle(self, event):
        """Включение/выключение насоса"""
        if self.btn_pump.GetBackgroundColour() == wx.Colour(0, 255, 0):  # Зеленый
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.startPUMP, 1)
            self.btn_pump.SetBackgroundColour(wx.Colour(255, 0, 0))  # Красный
            self.btn_pump.SetLabel("Стоп насоса")
            self.log_message("Насос запущен")
        else:
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.startPUMP, 0)
            self.btn_pump.SetBackgroundColour(wx.Colour(0, 255, 0))  # Зеленый
            self.btn_pump.SetLabel("Пуск насоса")
            self.log_message("Насос остановлен")

    def on_plus_frequency(self, event):
        """Увеличение частоты"""
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.plusPUMP, 1)
        wx.CallLater(self.controller.plus_minus_time, lambda:
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.plusPUMP, 0))
        self.log_message("Увеличена частота насоса")

    def on_minus_frequency(self, event):
        """Уменьшение частоты"""
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.minusPUMP, 1)
        wx.CallLater(self.controller.plus_minus_time, lambda:
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.minusPUMP, 0))
        self.log_message("Уменьшена частота насоса")

    def toggle_valve(self, btn, valve_num, valve_name):
        """Переключение состояния клапана"""
        if btn.GetBackgroundColour() == wx.Colour(255, 0, 0):  # Красный
            self.controller.send_705X_XX(7050, self.controller.adrDO, valve_num, 1)
            btn.SetBackgroundColour(wx.Colour(0, 255, 0))  # Зеленый
            self.log_message(f"{valve_name} открыт")
        else:
            self.controller.send_705X_XX(7050, self.controller.adrDO, valve_num, 0)
            btn.SetBackgroundColour(wx.Colour(255, 0, 0))  # Красный
            self.log_message(f"{valve_name} закрыт")

    def on_to_forward(self, event):
        self.toggle_valve(self.btn_to_forward, self.controller.valve1, "В клапан прямой")

    def on_to_revers(self, event):
        self.toggle_valve(self.btn_to_revers, self.controller.valve2, "В клапан обратный")

    def on_from_forward(self, event):
        self.toggle_valve(self.btn_from_forward, self.controller.valve3, "Из клапана прямой")

    def on_from_revers(self, event):
        self.toggle_valve(self.btn_from_revers, self.controller.valve4, "Из клапана обратный")

    def on_forward_test(self, event):
        """Запуск прямых испытаний"""
        if self.is_test_running:
            return

        self.is_test_running = True
        self.log_message("Начинаем прямые испытания")

        # Сброс всех состояний
        self.controller.stop_igra = False
        self.controller.plus_counter = 0
        self.controller.minus_counter = 0

        # Очистка графика
        self.line.set_data([], [])
        self.ax.set_xlim(0, self.controller.plus_step)
        self.canvas.draw()

        # Запуск теста в отдельном потоке
        threading.Thread(target=self.run_forward_test, daemon=True).start()

    def run_forward_test(self):
        """Выполнение прямых испытаний"""
        try:
            # Настройка системы
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.minusPUMP, 0)
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve1, 0)
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve2, 0)
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve3, 0)
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve4, 0)
            self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.startPUMP, 0)

            # Включение насоса и клапанов
            wx.CallAfter(self.on_pump_toggle, None)
            wx.CallAfter(self.on_to_forward, None)
            wx.CallAfter(self.on_from_forward, None)

            time.sleep(1)

            # Цикл испытаний
            for step in range(self.controller.plus_step):
                if self.controller.stop_igra:
                    break

                # Чтение давления
                pressure = self.controller.read_7019_2()
                if pressure is not None:
                    self.controller.plus_dim_press[step] = pressure

                    # Обновление GUI
                    wx.CallAfter(self.update_graph, step, pressure)
                    wx.CallAfter(self.log_message, f"Шаг {step}. Давление: {pressure:.2f} атм")

                # Увеличение частоты
                self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.plusPUMP, 1)
                time.sleep(self.controller.plus_minus_time / 1000)
                self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.plusPUMP, 0)

                time.sleep(self.controller.plus_step_time / 1000)

            # Анализ результатов
            max_pressure = max(self.controller.plus_dim_press[:self.controller.plus_counter])
            self.controller.fixed_open = max_pressure

            wx.CallAfter(self.log_message,
                         f"Зафиксировано давление открытия: {max_pressure:.2f} атм\n"
                         f"Зафиксировано давление закрытия: {self.controller.fixed_close:.2f} атм")

        except Exception as e:
            wx.CallAfter(self.log_message, f"Ошибка при испытаниях: {e}")
        finally:
            self.is_test_running = False
            wx.CallAfter(self.on_stop, None)

    def update_graph(self, step, pressure):
        """Обновление графика"""
        x_data = list(range(step + 1))
        y_data = self.controller.plus_dim_press[:step + 1]

        self.line.set_data(x_data, y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()

    def on_revers_test(self, event):
        """Обратные испытания"""
        self.log_message("Запуск обратных испытаний")
        # Реализация аналогична прямым испытаниям

    def on_stop(self, event):
        """Остановка всех процессов"""
        self.controller.stop_igra = True
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve1, 0)
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve2, 0)
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve3, 0)
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.valve4, 0)
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.startPUMP, 0)
        self.controller.send_705X_XX(7050, self.controller.adrDO, self.controller.minusPUMP, 1)

        # Сброс цветов кнопок
        wx.CallAfter(self.reset_buttons)
        self.log_message("Все процессы остановлены")

    def reset_buttons(self):
        """Сброс состояний кнопок"""
        self.btn_pump.SetBackgroundColour(wx.Colour(0, 255, 0))
        self.btn_pump.SetLabel("Пуск насоса")

        for btn in [self.btn_to_forward, self.btn_to_revers, self.btn_from_forward, self.btn_from_revers]:
            btn.SetBackgroundColour(wx.Colour(255, 0, 0))

    def on_close(self, event):
        """Обработка закрытия приложения"""
        self.timer.Stop()
        self.controller.disconnect()
        self.Destroy()


if __name__ == "__main__":
    app = wx.App()
    frame = MainFrame()
    frame.Show()
    app.MainLoop()