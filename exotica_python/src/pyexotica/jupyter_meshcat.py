__all__ = ["show"]

def show(url, height=50):
    try:
        import ipywidgets as widgets
        w=widgets.HTML(value='<iframe style="width:100%; height:'+str(height)+'em;border:0" src="'+url+'"></iframe>')
        w._ipython_display_()
    except ImportError:
        print('Jupyter Meshcat visualisation not supported!')
