from flask import Flask, render_template
import random

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/wearability')
def wearability():
    return f'<h4>{random.randint(80, 100)}</h4>'

if __name__ == '__main__':
      app.run(host='0.0.0.0', port=5052)
