from flask import Flask, render_template, send_file
import random

import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
#from simulation_modify_2 import safety_fig
from one_dof.python.one_dof_graph import safety, safety_fig, safety_fig2

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/wearability')
def wearability():
    return f'<h4>{random.randint(80, 100)}</h4>'

@app.route('/fig/safety')
def safety_img():
    #img = safety_fig(baselink_angle=0, device_angle=175)
    img = safety_fig(device_baselink_angle=0, device_elbow_angle=120)
    return send_file(img, mimetype='image/png')

@app.route('/fig2/safety')
def safety_img2():
    import time
    time.sleep(1)
    #img = safety_fig(baselink_angle=0, device_angle=175)
    img = safety_fig2(device_baselink_angle=0, device_elbow_angle=120)
    return send_file(img, mimetype='image/png')


@app.route('/safety')
def safety_txt():
    #result = safety(baselink_angle=0, device_angle=175)
    result = safety(device_baselink_angle=0, device_elbow_angle=120)
    #return f'<h4>{result[0]}, {result[1]}, {result[2]}, {result[3]}, {result[4]}</h4>'
    return f'<h4>{result}</h4>'


if __name__ == '__main__':
      app.run(host='0.0.0.0', port=5052)
