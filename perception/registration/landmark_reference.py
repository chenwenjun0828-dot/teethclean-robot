"""Display a saved numbered camera reference in an isolated Tk process."""
import sys
import tkinter as tk

from PIL import Image, ImageTk


def main():
    root = tk.Tk()
    root.title('CAMERA reference - match points 1, 2, 3... on SCAN')
    image = Image.open(sys.argv[1])
    image.thumbnail((min(1000, root.winfo_screenwidth() - 100),
                     min(800, root.winfo_screenheight() - 100)))
    photo = ImageTk.PhotoImage(image)
    tk.Label(root, text='Camera reference: match numbered points on the SCAN window.').pack()
    tk.Label(root, image=photo).pack()
    root.mainloop()


if __name__ == '__main__':
    main()
