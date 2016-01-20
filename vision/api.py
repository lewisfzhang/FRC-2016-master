from bottle import route, run, request, post
import C290

def server(ctl):
  @route('/configure', method="POST")
  def configure():
    exposure = ctl.setManualExposure(request.forms.get('exposure'))
    resp = {
      'exposure' : exposure
    }
    return resp

  run(host='localhost', port=8080)

if __name__ == "__main__":
  controller = C290.C290Ctl(0)
  server(controller)
