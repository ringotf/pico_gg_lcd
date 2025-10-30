import serial
import pygame
from PIL import Image

ser = serial.Serial('COM7', 115200, timeout=1)


pixels_in_scanline = 280 #300 #160
scanlines_in_active_area = 160 #144 #192 #144
scanlines_in_active_area_half = 84
scanlines_in_active_area_min = 100

scanlines_to_use = scanlines_in_active_area * 2#_min

frame_size = pixels_in_scanline * scanlines_to_use * 2

frame_size_to_use = frame_size# * 2#_min


lines_to_skip = 0 #16 #51

frame_size_to_use -= pixels_in_scanline * lines_to_skip

pygame.init()
screen = pygame.display.set_mode((pixels_in_scanline, scanlines_to_use - lines_to_skip), pygame.RESIZABLE)
pygame.display.set_caption("Pico GG Viewer")

pygame.font.init()
my_font = pygame.font.SysFont('Console', 22, True)
text_surface = my_font.render('Pico GG Viewer', False, (255,0,0))

screen.blit(text_surface, (0,0))


def convert_rgb444_to_rgb888(frame):
    rgb888 = bytearray()
    for i in range(0, len(frame), 2):
        pixel = int.from_bytes(frame[i:i+2],'little')
        r = (pixel >> 8) & 0xF #4 bit red
        g = (pixel >> 4) & 0xF #4 bit green
        b = pixel & 0xF #4 bit blue
        r = r * 255 // 15
        g = g * 255 // 15
        b = b * 255 // 15
        rgb888.extend([r,g,b])
    return bytes(rgb888)


frame_count = 0

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    ser.reset_input_buffer() #clear any partial data

    #print('Sending request byte...')

    #request a frame by sending enter
    ser.write(b'\x0A')
    ser.flush()
    #print('Sent request byte, waiting for frame...')

    frame = ser.read(frame_size_to_use)     

    #print("Frame length: ")
    #print(len(frame))

    if(len(frame) >= frame_size_to_use):   

        #with open('frame.bin', 'wb') as f:
        #    f.write(frame)

        rgb888 = convert_rgb444_to_rgb888(frame)
        img_pil = Image.frombytes('RGB', (pixels_in_scanline, scanlines_to_use - lines_to_skip), rgb888)
        img_data = img_pil.tobytes()

        #img_pil = Image.frombytes('RGB', (pixels_in_scanline, scanlines_in_active_area), frame, 'raw', 'BGR;16') #try BGR;16 if colours swap        
        #img_data = img_pil.convert('RGB').tobytes()

        #img = pygame.image.frombuffer(frame, (pixels_in_scanline, scanlines_in_active_area), 'RGB;16')
        img = pygame.image.frombuffer(img_data, (pixels_in_scanline, scanlines_to_use - lines_to_skip), 'RGB')

        screen.blit(img, (0, 0))
        
        frame_count+=1

        text_surface = my_font.render(str(frame_count), False, (255,0,0))

        screen.blit(text_surface, (0,0))

        pygame.display.flip()

        img_pil.save('frame.png')
                


ser.close()
pygame.quit()