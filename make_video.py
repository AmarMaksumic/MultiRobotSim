import os
import imageio.v2 as imageio

FRAME_DIR = "pygame_frames"
OUTPUT_VIDEO = "simulation.mp4"
FPS = 30

def make_video(frame_dir, output_path, fps=30):
    frames = sorted(
        [f for f in os.listdir(frame_dir) if f.endswith(".png")],
        key=lambda x: int(x.split("_")[1].split(".")[0])
    )
    images = [imageio.imread(os.path.join(frame_dir, f)) for f in frames]

    imageio.mimsave(output_path, images, fps=fps, codec='libx264', quality=8)
    print(f"Saved video to {output_path}")

if __name__ == "__main__":
    make_video(FRAME_DIR, OUTPUT_VIDEO, FPS)
