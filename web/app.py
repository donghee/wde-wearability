from flask import Flask, render_template, send_file, request
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from device_1DOF.python.wearability.device_1DOF_timegraph import timegraph, input_length
from device_1DOF.python.wearability.device_1DOF_totalgraph import totalgraph, totalscore

app = Flask(__name__)

@app.route('/')
def index():
    device = 'upper' if request.args.get('device') == None else request.args.get('device')
    task = 'arm' if request.args.get('task') == None else request.args.get('task')    
    body = 'body1' if request.args.get('body') == None else request.args.get('body')
    joint = 'joint1' if request.args.get('joint') == None else request.args.get('joint')
    url = f'/?device={device}&body={body}&joint={joint}'
    max_slide_size = input_length()
    subject = 'wearability'
    return render_template('index.html', url=url, subject=subject, body=body, device=device, task=task, max_slide_size=max_slide_size)

@app.route('/score')
def score():
    device = 'upper' if request.args.get('device') == None else request.args.get('device')
    body = 'body1' if request.args.get('body') == None else request.args.get('body')
    joint = 'joint1' if request.args.get('joint') == None else request.args.get('joint')
    wearability = round(totalscore() * 100.0, 1)
    return f'{wearability}'

@app.route('/fig/timegraph/<wear_case>/<line>')
def get_timegraph(wear_case, line):
    _wear_case = int(wear_case)
    _line = int(line)
    img = timegraph(_wear_case, _line)
    return send_file(img, mimetype='image/png')

@app.route('/fig/update/timegraph', methods=['POST'])
def update_timegraph():
    if request.method == "POST":
        print(request.form)
        timegraph_line = request.form['v']
        wear_case = request.form['case']
        return f"""
        <div class="flex justify-center py-12" id="range_value" hx-swap-oob="true" hx-swap="outerHTML">
        <img id="update-spinner" class="htmx-indicator" src="https://htmx.org/img/bars.svg"/ width=200>
        <img id="wearability-result-fig" src="fig/timegraph/{wear_case}/{timegraph_line}"/>
        </div>
        """

@app.route('/fig/totalgraph', methods=['GET'])
def get_totalgraph():
    img = totalgraph()
    return send_file(img, mimetype='image/png')

if __name__ == '__main__':
      app.run(host='0.0.0.0', port=5052, debug=True)
