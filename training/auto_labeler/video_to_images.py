#!/usr/bin/env python3
"""
Video to Images Extractor

A standalone GUI tool to extract frames from a video file to a folder of images.
Features:
- Frame navigation with scroll bar and buttons
- Motion detection to skip to frames with significant changes
- Start/end frame selection for export range
- Configurable output width
"""

import argparse
import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
from pathlib import Path
from tqdm import tqdm
import threading


class VideoToImagesApp:
    """GUI application for extracting frames from video."""

    def __init__(self, root: tk.Tk, video_path: str = None):
        self.root = root
        self.root.title("Video to Images Extractor")
        self.root.geometry("1200x800")

        # Video state
        self.video_path = video_path
        self.cap = None
        self.total_frames = 0
        self.current_frame = 0
        self.fps = 30.0
        self.video_width = 0
        self.video_height = 0

        # Display settings
        self.display_width = 800  # Display at smaller size for speed
        self.display_scale = 1.0

        # Export settings
        self.start_frame = 0
        self.end_frame = 0
        self.output_width = tk.IntVar(value=960)

        # Motion detection settings
        self.motion_threshold = tk.DoubleVar(value=5.0)
        self.prev_frame_gray = None

        # Build UI
        self._build_ui()

        # Load video if provided
        if video_path:
            self._load_video(video_path)

    def _build_ui(self):
        """Build the user interface."""
        # Main container
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill="both", expand=True)

        # Top bar - file selection
        file_frame = ttk.Frame(main_frame)
        file_frame.pack(fill="x", pady=(0, 10))

        ttk.Button(file_frame, text="Open Video", command=self._open_video).pack(
            side="left"
        )
        self.file_label = ttk.Label(file_frame, text="No video loaded")
        self.file_label.pack(side="left", padx=10)

        # Video display area
        display_frame = ttk.Frame(main_frame)
        display_frame.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(display_frame, bg="black", width=800, height=500)
        self.canvas.pack(fill="both", expand=True)

        # Frame slider
        slider_frame = ttk.Frame(main_frame)
        slider_frame.pack(fill="x", pady=10)

        self.frame_var = tk.IntVar(value=0)
        self.frame_slider = ttk.Scale(
            slider_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.frame_var,
        )
        self.frame_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.frame_slider.bind("<ButtonRelease-1>", self._on_slider_release)

        self.frame_label = ttk.Label(slider_frame, text="Frame: 0 / 0")
        self.frame_label.pack(side="left", padx=10)

        # Navigation buttons
        nav_frame = ttk.Frame(main_frame)
        nav_frame.pack(fill="x", pady=5)

        ttk.Button(nav_frame, text="⏮", width=4, command=self._go_first).pack(
            side="left", padx=2
        )
        ttk.Button(nav_frame, text="◀◀", width=4, command=lambda: self._step(-10)).pack(
            side="left", padx=2
        )
        ttk.Button(nav_frame, text="◀", width=4, command=lambda: self._step(-1)).pack(
            side="left", padx=2
        )
        ttk.Button(nav_frame, text="▶", width=4, command=lambda: self._step(1)).pack(
            side="left", padx=2
        )
        ttk.Button(nav_frame, text="▶▶", width=4, command=lambda: self._step(10)).pack(
            side="left", padx=2
        )
        ttk.Button(nav_frame, text="⏭", width=4, command=self._go_last).pack(
            side="left", padx=2
        )

        ttk.Separator(nav_frame, orient="vertical").pack(side="left", fill="y", padx=10)

        ttk.Button(nav_frame, text="Next Motion", command=self._next_motion).pack(
            side="left", padx=2
        )
        ttk.Label(nav_frame, text="Threshold:").pack(side="left", padx=(10, 2))
        ttk.Entry(nav_frame, textvariable=self.motion_threshold, width=6).pack(
            side="left"
        )

        # Export settings
        export_frame = ttk.LabelFrame(main_frame, text="Export Settings", padding=10)
        export_frame.pack(fill="x", pady=10)

        # Start/End frame selection
        range_frame = ttk.Frame(export_frame)
        range_frame.pack(fill="x", pady=5)

        ttk.Label(range_frame, text="Start Frame:").pack(side="left")
        self.start_var = tk.StringVar(value="0")
        self.start_entry = ttk.Entry(range_frame, textvariable=self.start_var, width=10)
        self.start_entry.pack(side="left", padx=5)
        ttk.Button(range_frame, text="Set Current", command=self._set_start).pack(
            side="left", padx=2
        )

        ttk.Label(range_frame, text="End Frame:").pack(side="left", padx=(20, 0))
        self.end_var = tk.StringVar(value="0")
        self.end_entry = ttk.Entry(range_frame, textvariable=self.end_var, width=10)
        self.end_entry.pack(side="left", padx=5)
        ttk.Button(range_frame, text="Set Current", command=self._set_end).pack(
            side="left", padx=2
        )

        self.range_label = ttk.Label(range_frame, text="(0 frames)")
        self.range_label.pack(side="left", padx=10)

        # Output settings
        output_frame = ttk.Frame(export_frame)
        output_frame.pack(fill="x", pady=5)

        ttk.Label(output_frame, text="Output Width:").pack(side="left")
        ttk.Entry(output_frame, textvariable=self.output_width, width=8).pack(
            side="left", padx=5
        )
        ttk.Label(output_frame, text="pixels (0 = original)").pack(side="left")

        ttk.Separator(output_frame, orient="vertical").pack(
            side="left", fill="y", padx=20
        )

        ttk.Button(
            output_frame, text="Export Images", command=self._export_images
        ).pack(side="left", padx=10)

        # Progress bar
        self.progress_var = tk.DoubleVar(value=0)
        self.progress_bar = ttk.Progressbar(
            export_frame, variable=self.progress_var, maximum=100
        )
        self.progress_bar.pack(fill="x", pady=5)

        self.status_label = ttk.Label(export_frame, text="Ready")
        self.status_label.pack(anchor="w")

        # Keyboard bindings
        self.root.bind("<Left>", lambda e: self._step(-1))
        self.root.bind("<Right>", lambda e: self._step(1))
        self.root.bind("<Shift-Left>", lambda e: self._step(-10))
        self.root.bind("<Shift-Right>", lambda e: self._step(10))
        self.root.bind("<Home>", lambda e: self._go_first())
        self.root.bind("<End>", lambda e: self._go_last())
        self.root.bind("<space>", lambda e: self._next_motion())

    def _open_video(self):
        """Open a video file dialog."""
        path = filedialog.askopenfilename(
            title="Select Video File",
            filetypes=[
                ("Video files", "*.mp4 *.avi *.mov *.mkv *.webm"),
                ("All files", "*.*"),
            ],
        )
        if path:
            self._load_video(path)

    def _load_video(self, path: str):
        """Load a video file."""
        if self.cap is not None:
            self.cap.release()

        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            messagebox.showerror("Error", f"Cannot open video: {path}")
            return

        self.video_path = path
        self.video_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        # Calculate display scale
        self.display_scale = min(self.display_width / self.video_width, 1.0)

        # Update UI
        self.file_label.config(
            text=f"{Path(path).name} ({self.video_width}x{self.video_height}, {self.total_frames} frames)"
        )
        self.frame_slider.config(to=max(0, self.total_frames - 1))

        # Set default export range
        self.start_frame = 0
        self.end_frame = self.total_frames - 1
        self.start_var.set("0")
        self.end_var.set(str(self.total_frames - 1))
        self._update_range_label()

        # Reset and show first frame
        self.current_frame = 0
        self.prev_frame_gray = None
        self._show_frame(0)

    def _show_frame(self, frame_idx: int):
        """Display a specific frame."""
        if self.cap is None:
            return

        frame_idx = max(0, min(frame_idx, self.total_frames - 1))
        self.current_frame = frame_idx

        # Seek and read frame
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if not ret:
            return

        # Store grayscale for motion detection
        self.prev_frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Resize for display
        if self.display_scale < 1.0:
            display_w = int(self.video_width * self.display_scale)
            display_h = int(self.video_height * self.display_scale)
            frame = cv2.resize(
                frame, (display_w, display_h), interpolation=cv2.INTER_AREA
            )

        # Convert to PhotoImage
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame_rgb)
        photo = ImageTk.PhotoImage(image)

        # Update canvas - center the image
        self.canvas.delete("all")
        self.canvas.update_idletasks()  # Ensure canvas dimensions are current
        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        center_x = canvas_w // 2
        center_y = canvas_h // 2
        self.canvas.create_image(center_x, center_y, anchor="center", image=photo)
        self.canvas._photo = photo  # Keep reference

        # Update labels
        self.frame_var.set(frame_idx)
        self.frame_label.config(text=f"Frame: {frame_idx} / {self.total_frames - 1}")

    def _on_slider_release(self, event):
        """Handle slider release."""
        frame_idx = int(self.frame_var.get())
        self._show_frame(frame_idx)

    def _step(self, delta: int):
        """Step forward/backward by delta frames."""
        self._show_frame(self.current_frame + delta)

    def _go_first(self):
        """Go to first frame."""
        self._show_frame(0)

    def _go_last(self):
        """Go to last frame."""
        self._show_frame(self.total_frames - 1)

    def _next_motion(self):
        """Skip to next frame with significant motion."""
        if self.cap is None or self.prev_frame_gray is None:
            return

        threshold = self.motion_threshold.get()
        start_frame = self.current_frame + 1

        self.status_label.config(text="Searching for motion...")
        self.root.update()

        for frame_idx in tqdm(
            range(start_frame, self.total_frames),
            total=(self.total_frames - start_frame),
        ):
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Calculate frame difference
            diff = cv2.absdiff(self.prev_frame_gray, gray)
            mean_diff = np.mean(diff)

            if mean_diff > threshold:
                self.status_label.config(text=f"Motion detected (diff={mean_diff:.2f})")
                self._show_frame(frame_idx)
                return

            self.prev_frame_gray = gray

            # Update progress occasionally
            if frame_idx % 100 == 0:
                self.status_label.config(text=f"Searching... frame {frame_idx}")
                self.root.update()

        self.status_label.config(text="No more motion detected")
        messagebox.showinfo("Motion Search", "No more frames with motion detected.")

    def _set_start(self):
        """Set current frame as start frame."""
        self.start_frame = self.current_frame
        self.start_var.set(str(self.current_frame))
        self._update_range_label()

    def _set_end(self):
        """Set current frame as end frame."""
        self.end_frame = self.current_frame
        self.end_var.set(str(self.current_frame))
        self._update_range_label()

    def _update_range_label(self):
        """Update the frame range label."""
        try:
            start = int(self.start_var.get())
            end = int(self.end_var.get())
            count = max(0, end - start + 1)
            self.range_label.config(text=f"({count} frames)")
        except ValueError:
            self.range_label.config(text="(invalid)")

    def _export_images(self):
        """Export frames to images."""
        if self.cap is None:
            messagebox.showerror("Error", "No video loaded")
            return

        try:
            start = int(self.start_var.get())
            end = int(self.end_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid start/end frame values")
            return

        if start > end:
            messagebox.showerror("Error", "Start frame must be <= end frame")
            return

        output_width = self.output_width.get()

        # Select output directory (start in same directory as video)
        initial_dir = str(Path(self.video_path).parent) if self.video_path else None
        output_dir = filedialog.askdirectory(
            title="Select Output Directory", initialdir=initial_dir
        )
        if not output_dir:
            return

        # Create subfolder named after the video file (without extension)
        video_name = Path(self.video_path).stem
        output_path = Path(output_dir) / video_name
        output_path.mkdir(parents=True, exist_ok=True)

        # Calculate output dimensions
        if output_width > 0 and output_width != self.video_width:
            scale = output_width / self.video_width
            out_w = output_width
            out_h = int(self.video_height * scale)
        else:
            out_w = self.video_width
            out_h = self.video_height
            scale = 1.0

        # Export in background thread
        def export_thread():
            total = end - start + 1
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, start)

            for i, frame_idx in enumerate(range(start, end + 1)):
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Resize if needed
                if scale != 1.0:
                    frame = cv2.resize(
                        frame, (out_w, out_h), interpolation=cv2.INTER_AREA
                    )

                # Save frame
                filename = output_path / f"frame_{frame_idx:06d}.jpg"
                cv2.imwrite(str(filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

                # Update progress
                progress = (i + 1) / total * 100
                self.progress_var.set(progress)
                self.status_label.config(
                    text=f"Exporting frame {frame_idx} ({i + 1}/{total})"
                )
                self.root.update()

            self.status_label.config(
                text=f"Done! Exported {total} frames to {output_dir}"
            )
            messagebox.showinfo(
                "Export Complete", f"Exported {total} frames to:\n{output_dir}"
            )

        threading.Thread(target=export_thread, daemon=True).start()

    def close(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Extract frames from video to images")
    parser.add_argument("video", nargs="?", help="Path to video file")
    args = parser.parse_args()

    root = tk.Tk()
    app = VideoToImagesApp(root, args.video)

    def on_close():
        app.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
