from klampt import *
import numpy as np
from PIL import Image,ImageMath
import time

#GLUT does better extracting depth buffer
#vis.init('GLUT')

def test_screenshot():
    def take_screenshot1():
        print("Saving as temp1.jpg, temp2.png")
        screenshot1,screenshot2 = vis.screenshot('Image',True)
        screenshot1.save("temp1.jpg","JPEG")
        scaled = ImageMath.eval("a/8*255",a=screenshot2)
        print("Depthmin/max",scaled.getextrema())
        scaled = scaled.convert("L")
        print("Converted to L: min/max",scaled.getextrema())
        scaled.save("temp2.png","PNG")

    def take_screenshot2():
        print("Saving as temp3.jpg, temp4.png")
        screenshot1,screenshot2 = vis.screenshot('numpy',True)
        screenshot1 = Image.fromarray(screenshot1)
        if screenshot1.mode == 'RGBA':
            screenshot1 = screenshot1.convert("RGB")
        screenshot1.save("temp3.jpg","JPEG")
        scaled = (screenshot2/8*255).clip(0,255)
        print("Depth min",scaled.min(),"max",scaled.max())
        im = Image.fromarray(scaled.astype(np.uint8))
        print("Converted to Image: min/max",im.getextrema())
        im.save("temp4.png","PNG")
    
    def take_screenshot3():
        print("Showing as Matplotlib plots")
        from matplotlib import pyplot as plt
        screenshot1,screenshot2 = vis.screenshot('numpy',True)
        fig,axs = plt.subplots(1,2,figsize=(14,4))
        axs[0].imshow(screenshot1)
        axs[1].imshow(screenshot2)
        plt.show()
        time.sleep(1.0)
            
    vis.addAction(take_screenshot1,'Screenshot Image','1')
    vis.addAction(take_screenshot2,'Screenshot numpy','2')
    vis.addAction(take_screenshot3,'Screenshot numpy + matplotlib','3')
    
    vis.show()

    #testing initial screen shot
    screenshot = vis.screenshot('Image')
    screenshot.save("temp0.jpg","JPEG")
    vis.spin(float('inf'))
    vis.kill()

if __name__ == '__main__':
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    r = w.robot(0)

    #show edges of a link
    r.link(0).appearance().setDraw(Appearance.EDGES,True)
    r.link(0).appearance().setColor(Appearance.EDGES,1,1,1,0.2)

    vis.add("world",w)
    test_screenshot()