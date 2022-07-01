import json
import os
import serial
import numpy as np
import time
from bitstring import Bits
from scipy.signal import savgol_filter
from pypot.dynamixel import Dxl320IO, get_available_ports

ser = serial.Serial()
inputs={}

from flask import Flask, render_template, request

basedir = os.path.abspath(os.path.dirname(__file__))
app = Flask(__name__)
progress_count=0

def init():
    global io
    global servo
    servo=1
    ports = get_available_ports()

    if not len(ports):
        print('no port found!')

    print('ports found!', ports)

    #num_port = int(input('numero du port ?'))
    num_port = 0

    io = Dxl320IO(ports[num_port])

    io.set_moving_speed({servo: 0})
    return io

# Gain pid
def Reglage_gain(servo,P_gain,I_gain,D_gain):
    io.set_pid_gain({servo: [P_gain, I_gain, D_gain]})

def Essai_freq(servo,amplitude,frequence,nb_periode,num_fig):
    global progress_count

    AMP = amplitude
    FREQ = frequence
    nb_per = nb_periode
    duree = nb_per*2*np.pi/FREQ


    x_time_sin = []
    y_cons_sin = []
    y_reel_sin = []


    io.set_goal_position({servo: 0})
    time.sleep(2)

    t0 = time.time()
    duree = nb_per* 1/FREQ
    while True:
        t = time.time()
        if (t - t0) > duree:
            break
        progress_count=100*(t-t0)/duree
        poss = AMP * np.sin(2 * np.pi * FREQ * (t-t0))

        io.set_goal_position({servo: poss})

        x_time_sin.append(t-t0)
        y_cons_sin.append(poss)
        y_reel_sin.append(io.get_present_position((servo, ))[0])
    return (x_time_sin, y_cons_sin,y_reel_sin)

@app.route('/')
def index():
    global input, output
    gp1_sliders=[
        {'name':'nb_periodes','label':'Nombre de périodes','target': '#nb_periodes','val': [i for i in range(10)],'set': [1]},
        {'name':'amplitude','label':'Amplitude (°)','target': '#amplitude','val': [i for i in range(1,180)],'set': [45]},
        {'name':'periode','label':'Période (s)','target': '#periode','val': [i/20 for i in range(1,40)],'set': [1]},
        {'name':'kp','label':'Kp','target': '#kp','val': [i/10 for i in range(0,320)], 'set': [1]},
        {'name':'ki','label':'Ki','target': '#ki','val': [i/10 for i in range(0,320)], 'set': [0]},
        {'name':'kd','label':'Kd','target': '#kd','val': [i/10 for i in range(0,320)], 'set': [0]},
    ]
    gp1_radios=[
         {'name':'fonction','label':'Fonction sin/cos','convertpython': None,\
          'choices': [{'name':'sin','label':'Sinus','checked':True},{'name':'cos','label':'Cosinus','checked':False}]},
    ]
    gp1_buttons=[
         {'name':'start','label':'Démarrer mesure','action':'send_exp();'},
    ]
    outputs=[[{'name':'reel','label': 'Réel', 'color': '#bf4510','yaxe':'amp', 'borderDash':[]},
              {'name':'consigne','label': 'Consigne', 'color': '#10bf4a','yaxe':'amp', 'borderDash':[]}
              ]
             ]
    inputs['sliders']=[gp1_sliders]
    inputs['radios']=[gp1_radios]
    inputs['checks']=[]
    buttons=[gp1_buttons]
    return render_template('line_chart.html', inputs=inputs, buttons=buttons, outputs=outputs)

@app.route('/process', methods=['POST', 'GET'])
def calculation():
    if request.method == "POST":
        data_input = request.get_json()
        variables={}
        for elts in inputs.values():
            for gp_elts in elts:
                for elt in gp_elts:
                    if elt['convertpython']:
                        variables[elt['name']]=elt['convertpython'](data_input[elt['name']])
                    else:
                        variables[elt['name']]=data_input[elt['name']]
        return json.dumps('')

@app.route('/send_exp', methods=['POST', 'GET'])
def send_exp():
    global progress_count
    progress_count=0
    if request.method == "POST":
        servo=1
        data_input = request.get_json()
        periode=float(data_input['periode'])
        nb_periodes=float(data_input['nb_periodes'])
        amplitude=float(data_input['amplitude'])
        kp=float(data_input['kp'])
        ki=float(data_input['ki'])
        kd=float(data_input['kd'])
        fonction=data_input['fonction']
        init()
        Reglage_gain(servo,kp,ki,kd)
        t, y1, y2=Essai_freq(servo,amplitude,1/periode,nb_periodes,0)
        keys_list=['x_time','reel','consigne']
        results=[dict(zip(keys_list, [t[i],y1[i],y2[i]])) for i in range(len(t))]
        progress_count=0
        return json.dumps(results)

@app.route('/progress/1')
def progress():
    return str(progress_count)

if __name__ == "__main__":
    app.run(debug=True)
