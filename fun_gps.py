

import os

if 'LOGNAME' in os.environ.keys():
    import micropip
    await micropip.install('ipywidgets')
    await micropip.install('ipyleaflet')     


import gps as gps
import json
import ipywidgets as widgets
from IPython.display import display, Javascript
from ipyleaflet import Map, TileLayer, FullScreenControl, ScaleControl, WidgetControl

name = None

from IPython.display import display, Javascript

def rerun_cell():
    display(Javascript('IPython.notebook.execute_cell()'))

# Llamar a la función rerun_cell para ejecutar la celda actual de nuevo
# rerun_cell()

def update_output_label():
    return f"""
        <span style="color: lime;">&nbsp;&nbsp;&nbsp;ZERO:</span><span style="color: bisque;"> {gps.ZERO}</span>&nbsp;&nbsp;&nbsp;
        <span style="color: coral;">INCLINATION:</span><span style="color: bisque;"> {gps.INCLINATION}</span>&nbsp;&nbsp;&nbsp;
        <span style="color: cyan;">R_TER:</span><span style="color: bisque;"> {gps.R_TER}</span>
    """

def terminal_selector():
    global name
    
    with open('geodata.json', 'r') as file:
        datos = json.load(file)
        terminales = ['-'] + sorted(list(datos.keys()))
    
    terminales_widget = widgets.Dropdown(
        options=terminales,
        value=name, 
        description='Terminal:',
        disabled=False,
        layout=widgets.Layout(width='15%')
    )
    
    output_label = widgets.HTML(value="")
    if name is not None:
        output_label.value = update_output_label()
        

    def on_terminal_change(change):
        global name
        name = change.new 

        if name != '-':
            gps.load_geodata(name)
            gps.load_reference(name)
            output_label.value = update_output_label()
            rerun_cell()
        else:
            gps.ZERO, gps.INCLINATION, gps.R_TER = None, None, None
            output_label.value = ""
    
    terminales_widget.observe(on_terminal_change, names='value')

    display(widgets.HBox([terminales_widget, output_label]))


def create_map():

    if gps.TERMINAL is None:
        print('SELECCIONA TERMINAL')
        return

    coordinates_text = widgets.Text(
    value="Coordenadas: (0, 0)",
    description="Clic en mapa:"
    )
    
    # Crear capa base de Google Satellite
    gmaps = TileLayer(
        url="https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
        attribution="&copy; Google",
        max_zoom=22,
        min_zoom=0,
        opacity=1
    )
    
    # Crear el mapa con el centro y nivel de zoom inicial
    m = Map(layers=[gmaps], center=gps.MED, zoom=gps.ZOOM, scroll_wheel_zoom=True)

    # Crear un widget HTML para mostrar el texto en el mapa (esto simula un div)
    text_html = widgets.HTML(value="<div style='font-size: 20px; color: blue;'>Haz clic en el mapa para ver las coordenadas</div>")

    # Crear un WidgetControl para colocar el HTML en el mapa
    html_control = WidgetControl(widget=text_html, position='bottomleft')

    # Añadir el control de pantalla completa
    m.add_control(html_control)
    m.add(FullScreenControl())
    m.add(ScaleControl(position='bottomright'))
    
    def handle_map_interaction(**kwargs):

        lat = round(kwargs['coordinates'][0], 6)
        lng = round(kwargs['coordinates'][1], 6)

        p_gps = [lat, lng]
        p_xy  = gps.GPS2XY(p_gps, 2)
        x = p_xy[0]
        y = p_xy[1]
    
        if kwargs['type'] == 'mousemove':
            
            text_html.value = f"""
            <div style='font-size: 12px; color: #333; text-align: left; background: rgba(255, 255, 255, 0.7); padding: 2px 10px; border-radius: 5px; line-height: 1.2; width: 150px;'>
                🔵 {str(lat)}, {str(lng)} <br>
                🔴 {str(x)}, {str(y)}
            </div>
            """
        elif kwargs['type'] == 'click':
            coordinates_text.value = str(p_gps)

    m.on_interaction(handle_map_interaction)
    
    # Mostrar el mapa
    display(coordinates_text)
    return(m)



    
