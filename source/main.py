from source.function import reward_function


def main():
    params = {
        'is_crashed': False,
        'is_offtrack': False,
        'is_reversed': False,
        'speed': 1.0,
        'progress': 5.0,
        'track_width': 0.6,
        'heading': 0.0,
        'closest_waypoints': [3, 4],
        'x': 2.5,
        'y': 0.67,
        'steering_angle': 0.0,
        'steps': 0.0,
        'waypoints': [[2.9685466767304676, 0.3736381785611765],
                      [3.376694115196499, 0.6891184741673322],
                      [3.5728138542999743, 0.6950269601678832],
                      [4.175480216771129, 0.6840495560783193],
                      [4.491794389504936, 0.7401760259354242],
                      [4.54994872433359, 0.7525950010523018],
                      [5.226482797880217, 0.7437612285901243],
                      [5.416255183343521, 0.692623556225787],
                      [5.742516007240561, 0.6056313110625412],
                      [6.2062900143142405, 0.635845167711337],
                      [6.417974651778329, 0.7047409380402346],
                      [6.464302429922779, 0.712125858637163],
                      [6.6672287553338405, 0.8320646652734485],
                      [6.764774832845346, 0.8984965203326349],
                      [6.9188633342492185, 1.0354758926026022],
                      [6.993677415068423, 1.122757091589803],
                      [7.064482233397748, 1.222061557250806],
                      [7.201959436145865, 1.7479760429718136],
                      [7.247423546445465, 1.7982057170313757],
                      [7.188208570805801, 1.8168143673834505],
                      [7.025260162291785, 2.199327497769052],
                      [6.968641431151495, 2.3153429125813436],
                      [6.740811140159384, 2.6319461999521696],
                      [6.6070964499892, 2.7654051247869744],
                      [6.134902438230396, 3.041952713625727],
                      [5.9459106925648, 3.113336385287274],
                      [5.7513648499934495, 3.1391951153808266],
                      [5.708101289086245, 2.582050778866524],
                      [5.228898865876017, 3.0690557535191396],
                      [5.091209303968753, 3.047156894727873],
                      [5.055475786023062, 2.9266674973536593],
                      [5.012984775303909, 3.0346676276096205],
                      [4.669631772221663, 3.033620450434691],
                      [4.303369248625842, 3.189000714325304],
                      [4.0969656893245805, 3.3500182113567174],
                      [4.002885223677998, 3.4489004084365975],
                      [3.7927110651911735, 3.7045827237462756],
                      [3.6870251899412274, 3.841708321654534],
                      [3.539245850094074, 4.026824678723148],
                      [3.3041218772114833, 4.253314277848087],
                      [3.2235278535259915, 4.30999454820976],
                      [3.1366581616989575, 4.35888872241245],
                      [3.0219564879476404, 4.4074151577683836],
                      [2.909105565221921, 4.4403495048092445],
                      [2.810368459912069, 4.770754185696905],
                      [2.5569453366700077, 4.474772342161224],
                      [2.297774512626471, 4.435532919751433],
                      [2.0562198417423883, 4.369009730744065],
                      [1.8063296869039034, 4.302442497665341],
                      [1.350115656967678, 4.142818656449957],
                      [1.1027409142805962, 4.355575409229726],
                      [0.946325809815935, 3.822435727542167],
                      [0.8194826695130404, 3.620561589993138],
                      [0.8015505014567665, 2.746956835107409],
                      [0.8347727877174008, 2.6858524616879054],
                      [0.9107954576162325, 2.558220236577064],
                      [0.9831302434110358, 2.450776718761284],
                      [1.2101261525769067, 2.0954712823566664],
                      [1.2481405712569473, 2.016048568242944],
                      [1.315445638701804, 1.7771061445892964],
                      [1.4222153923934953, 1.250414700600402],
                      [1.2841362349422683, 1.1655870550755763],
                      [1.2878091634183724, 1.1190260817689115],
                      [1.4637267062667323, 1.1400796914032791],
                      [1.3679193493006816, 0.8914076290528806],
                      [1.5355271129845485, 0.996748990256467],
                      [1.464473454255551, 0.785135238003985],
                      [1.9009937145581672, 0.6109098568442695],
                      [2.707405186662496, 0.3932314638314363],
                      [2.8389740478559014, 0.38388247153447946]]
    }
    score = reward_function(params)
    print("score: " + str(score))


if __name__ == "__main__":
    main()
