
  # DrawBot Web App

  This is a code bundle for DrawBot Web App. The original project is available at https://www.figma.com/design/oLELNfNARINjkW4a7Yk8Eo/Robot-Companion-Web-App.

  ## Running the code

From the project root:
  Run `npm i` to install the dependencies.
  Run `npm run dev` to start the development server.
  Run `python app.py` to start the flask server.

  ## Configuring Paddle OCR
  1) Ensure your python installation <=3.14.
  2) If Windows, GPU is available to use.
  3) Install paddleocr
  4) If Windows, install:
    python -m pip install paddlepaddle-gpu==3.2.2 -i https://www.paddlepaddle.org.cn/packages/stable/cu129/
  5) Otherwise, install: 
    paddlepaddle
  6) You're good to go!
  