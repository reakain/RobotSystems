from adc import ADC
import math

class Grayscale_Module(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref
        self.old_data = self.get_grayscale_data()
        self.old_idx = 0


    def get_line_status(self,fl_list):

        if fl_list[0] > self.ref and fl_list[1] > self.ref and fl_list[2] > self.ref:
            return 'stop'
            
        elif fl_list[1] <= self.ref:
            return 'forward'
        
        elif fl_list[0] <= self.ref:
            return 'right'

        elif fl_list[2] <= self.ref:
            return 'left'

    def get_line_pos(self, target = "dark", sensitivity = 1.5):
        current_data = self.get_grayscale_data()
        # Do some nonsense to get back. Full left is -1, full right is 1
        # if target is dark, look for lower values, if target light, look for lighter values
        if target == "dark":
            # Get minimum value
            idx = current_data.index(math.min(current_data))
        else:
            # Get minimum value
            idx = current_data.index(math.max(current_data))

        if idx == 0:
            return -1
        elif idx == 1:
            return 0
        elif idx == 2:
            return 1

        self.old_data = current_data


    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

if __name__ == "__main__":
    import time
    GM = Grayscale_Module(950)
    while True:
        print(GM.get_grayscale_data())
        time.sleep(1)
