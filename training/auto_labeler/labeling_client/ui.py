"""Interactive labeling UI using Tkinter."""

import threading
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Callable, Dict, List, Optional, Tuple
from PIL import Image, ImageTk

from .api_client import LabelingAPIClient, ObjectLabel


class LabelingUI:
    """Interactive UI for video labeling."""

    def __init__(self, server_url: str):
        """
        Initialize the labeling UI.

        Args:
            server_url: URL of the labeling server
        """
        self.client = LabelingAPIClient(server_url)

        # UI state
        self.current_frame = 0
        self.current_label: Optional[ObjectLabel] = None
        self.add_mode = True  # True = add points (label=1), False = remove (label=0)
        self.show_masks = True
        self.show_points = True
        self.is_propagating = False

        # Image display state
        self._current_image: Optional[Image.Image] = None
        self._photo_image: Optional[ImageTk.PhotoImage] = None
        self._display_scale = 1.0

        # Create main window
        self.root = tk.Tk()
        self.root.title("Video Labeling Tool")
        self.root.geometry("1400x900")

        # Configure grid
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)

        self._create_ui()
        self._bind_keys()

    def _create_ui(self):
        """Create the UI components."""
        # Main frame
        main_frame = ttk.Frame(self.root, padding=5)
        main_frame.grid(row=0, column=0, sticky="nsew")
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Canvas for image display
        self.canvas_frame = ttk.Frame(main_frame)
        self.canvas_frame.grid(row=0, column=0, sticky="nsew")
        self.canvas_frame.columnconfigure(0, weight=1)
        self.canvas_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            self.canvas_frame,
            bg="#333333",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        # Navigation bar at bottom
        nav_frame = ttk.Frame(main_frame)
        nav_frame.grid(row=1, column=0, sticky="ew", pady=5)

        # Frame slider
        self.frame_var = tk.IntVar(value=0)
        self.frame_slider = ttk.Scale(
            nav_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.frame_var,
        )
        self.frame_slider.pack(side="left", fill="x", expand=True, padx=5)
        self.frame_slider.bind("<ButtonRelease-1>", self._on_slider_release)

        # Frame counter
        self.frame_label = ttk.Label(nav_frame, text="Frame: 0 / 0")
        self.frame_label.pack(side="left", padx=10)

        # Control panel on right
        control_frame = ttk.Frame(self.root, padding=10, width=300)
        control_frame.grid(row=0, column=1, sticky="ns")
        control_frame.grid_propagate(False)

        self._create_control_panel(control_frame)

    def _create_control_panel(self, parent):
        """Create the control panel on the right side."""
        # Status section
        status_frame = ttk.LabelFrame(parent, text="Status", padding=5)
        status_frame.pack(fill="x", pady=5)

        self.status_label = ttk.Label(status_frame, text="Not connected")
        self.status_label.pack(anchor="w")

        self.progress_var = tk.DoubleVar(value=0)
        self.progress_bar = ttk.Progressbar(
            status_frame,
            variable=self.progress_var,
            mode="determinate",
        )
        self.progress_bar.pack(fill="x", pady=5)

        # Labels section
        labels_frame = ttk.LabelFrame(parent, text="Object Labels", padding=5)
        labels_frame.pack(fill="x", pady=5)

        self.label_var = tk.StringVar()
        self.label_buttons: Dict[int, ttk.Radiobutton] = {}
        self.labels_container = ttk.Frame(labels_frame)
        self.labels_container.pack(fill="x")

        # Point mode section
        mode_frame = ttk.LabelFrame(parent, text="Point Mode", padding=5)
        mode_frame.pack(fill="x", pady=5)

        self.mode_var = tk.StringVar(value="add")
        ttk.Radiobutton(
            mode_frame,
            text="➕ Add (Include)",
            variable=self.mode_var,
            value="add",
            command=self._on_mode_change,
        ).pack(anchor="w")
        ttk.Radiobutton(
            mode_frame,
            text="➖ Remove (Exclude)",
            variable=self.mode_var,
            value="remove",
            command=self._on_mode_change,
        ).pack(anchor="w")

        # Point actions section
        actions_frame = ttk.LabelFrame(parent, text="Point Actions", padding=5)
        actions_frame.pack(fill="x", pady=5)

        btn_frame = ttk.Frame(actions_frame)
        btn_frame.pack(fill="x")

        ttk.Button(
            btn_frame,
            text="↩ Undo",
            command=self._on_undo,
        ).pack(side="left", padx=2)

        ttk.Button(
            btn_frame,
            text="🗑 Clear",
            command=self._on_clear,
        ).pack(side="left", padx=2)

        ttk.Button(
            btn_frame,
            text="🔍 Preview",
            command=self._on_preview,
        ).pack(side="left", padx=2)

        # Propagate section
        prop_frame = ttk.LabelFrame(parent, text="Propagation", padding=5)
        prop_frame.pack(fill="x", pady=5)

        self.propagate_btn = ttk.Button(
            prop_frame,
            text="▶ Propagate Masks",
            command=self._on_propagate,
        )
        self.propagate_btn.pack(fill="x")

        # Navigation section
        nav_frame = ttk.LabelFrame(parent, text="Navigation", padding=5)
        nav_frame.pack(fill="x", pady=5)

        nav_btn_frame = ttk.Frame(nav_frame)
        nav_btn_frame.pack(fill="x", pady=2)

        ttk.Button(
            nav_btn_frame,
            text="⏮",
            width=4,
            command=lambda: self._go_to_frame(0),
        ).pack(side="left", padx=1)

        ttk.Button(
            nav_btn_frame,
            text="◀◀",
            width=4,
            command=lambda: self._jump_frames(-10),
        ).pack(side="left", padx=1)

        ttk.Button(
            nav_btn_frame,
            text="◀",
            width=4,
            command=lambda: self._jump_frames(-1),
        ).pack(side="left", padx=1)

        ttk.Button(
            nav_btn_frame,
            text="▶",
            width=4,
            command=lambda: self._jump_frames(1),
        ).pack(side="left", padx=1)

        ttk.Button(
            nav_btn_frame,
            text="▶▶",
            width=4,
            command=lambda: self._jump_frames(10),
        ).pack(side="left", padx=1)

        ttk.Button(
            nav_btn_frame,
            text="⏭",
            width=4,
            command=self._go_to_last_frame,
        ).pack(side="left", padx=1)

        # Jump buttons
        jump_frame = ttk.Frame(nav_frame)
        jump_frame.pack(fill="x", pady=5)

        ttk.Button(
            jump_frame,
            text="⏪ Prev Manual",
            command=self._go_prev_manual,
        ).pack(side="left", padx=2)

        ttk.Button(
            jump_frame,
            text="Next Manual ⏩",
            command=self._go_next_manual,
        ).pack(side="left", padx=2)

        ttk.Button(
            nav_frame,
            text="🔎 Go to Next Unlabeled",
            command=self._go_next_unlabeled,
        ).pack(fill="x", pady=2)

        # Display options
        display_frame = ttk.LabelFrame(parent, text="Display Options", padding=5)
        display_frame.pack(fill="x", pady=5)

        self.show_masks_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            display_frame,
            text="Show Masks",
            variable=self.show_masks_var,
            command=self._update_display,
        ).pack(anchor="w")

        self.show_points_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            display_frame,
            text="Show Points",
            variable=self.show_points_var,
            command=self._update_display,
        ).pack(anchor="w")

        # Frame info
        info_frame = ttk.LabelFrame(parent, text="Frame Info", padding=5)
        info_frame.pack(fill="x", pady=5)

        self.info_text = tk.Text(info_frame, height=6, width=30, state="disabled")
        self.info_text.pack(fill="x")

        # Save button
        ttk.Button(
            parent,
            text="💾 Save",
            command=self._on_save,
        ).pack(fill="x", pady=5)

    def _bind_keys(self):
        """Bind keyboard shortcuts."""
        self.root.bind("<Left>", lambda e: self._jump_frames(-1))
        self.root.bind("<Right>", lambda e: self._jump_frames(1))
        self.root.bind("<Shift-Left>", lambda e: self._jump_frames(-10))
        self.root.bind("<Shift-Right>", lambda e: self._jump_frames(10))
        self.root.bind("<Home>", lambda e: self._go_to_frame(0))
        self.root.bind("<End>", lambda e: self._go_to_last_frame())
        self.root.bind("<Control-z>", lambda e: self._on_undo())
        self.root.bind("<space>", lambda e: self._on_preview())
        self.root.bind("<Return>", lambda e: self._on_propagate())
        self.root.bind("<Escape>", lambda e: self._toggle_mode())

        # Number keys for label selection
        for i in range(1, 10):
            self.root.bind(
                str(i), lambda e, idx=i: self._select_label_by_index(idx - 1)
            )

    def _toggle_mode(self):
        """Toggle between add and remove mode."""
        if self.mode_var.get() == "add":
            self.mode_var.set("remove")
        else:
            self.mode_var.set("add")
        self._on_mode_change()

    def _on_mode_change(self):
        """Handle point mode change."""
        self.add_mode = self.mode_var.get() == "add"

    def _select_label_by_index(self, index: int):
        """Select a label by its index."""
        labels = self.client.config.object_labels
        if 0 <= index < len(labels):
            self.label_var.set(labels[index].name)
            self.current_label = labels[index]

    def _populate_labels(self):
        """Populate label selection buttons."""
        # Clear existing
        for widget in self.labels_container.winfo_children():
            widget.destroy()
        self.label_buttons.clear()

        for i, label in enumerate(self.client.config.object_labels):
            # Create color preview
            color = label.color
            color_hex = f"#{color[0]:02x}{color[1]:02x}{color[2]:02x}"

            frame = ttk.Frame(self.labels_container)
            frame.pack(fill="x", pady=2)

            # Color indicator
            color_label = tk.Label(
                frame,
                bg=color_hex,
                width=2,
                relief="solid",
            )
            color_label.pack(side="left", padx=2)

            # Radio button
            rb = ttk.Radiobutton(
                frame,
                text=f"{i + 1}: {label.name}",
                variable=self.label_var,
                value=label.name,
                command=lambda l=label: self._on_label_select(l),
            )
            rb.pack(side="left", fill="x", expand=True)
            self.label_buttons[label.id] = rb

        # Select first label by default
        if self.client.config.object_labels:
            first = self.client.config.object_labels[0]
            self.label_var.set(first.name)
            self.current_label = first

    def _on_label_select(self, label: ObjectLabel):
        """Handle label selection."""
        self.current_label = label

    def _on_slider_release(self, event):
        """Handle slider release - only load frame when user lets go."""
        frame_idx = int(self.frame_var.get())
        if frame_idx != self.current_frame:
            self._go_to_frame(frame_idx)

    def _on_canvas_resize(self, event):
        """Handle canvas resize."""
        self._update_display()

    def _on_canvas_click(self, event):
        """Handle canvas click to add a point."""
        if self.current_label is None:
            messagebox.showwarning("No Label", "Please select a label first.")
            return

        if self._current_image is None:
            return

        # Convert canvas coordinates to image coordinates
        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        img_w, img_h = self._current_image.size

        # Calculate image position on canvas (centered)
        scale = min(canvas_w / img_w, canvas_h / img_h)
        scaled_w = int(img_w * scale)
        scaled_h = int(img_h * scale)
        offset_x = (canvas_w - scaled_w) // 2
        offset_y = (canvas_h - scaled_h) // 2

        # Check if click is within image
        x = event.x - offset_x
        y = event.y - offset_y

        if x < 0 or x >= scaled_w or y < 0 or y >= scaled_h:
            return

        # Convert to original image coordinates
        img_x = x / scale
        img_y = y / scale

        # Determine label based on mode
        label = 1 if self.add_mode else 0

        # Add point via API
        try:
            self.client.add_point(
                frame_idx=self.current_frame,
                x=img_x,
                y=img_y,
                label=label,
                obj_id=self.current_label.id,
            )
            self._update_display()
            self._update_frame_info()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to add point: {e}")

    def _update_display(self):
        """Update the displayed frame."""
        if not hasattr(self.client, "_config") or self.client._config is None:
            return

        try:
            # Get frame from server
            self._current_image = self.client.get_frame(
                self.current_frame,
                include_masks=self.show_masks_var.get(),
                include_points=self.show_points_var.get(),
            )

            # Scale to fit canvas
            canvas_w = self.canvas.winfo_width()
            canvas_h = self.canvas.winfo_height()

            if canvas_w < 10 or canvas_h < 10:
                return

            img_w, img_h = self._current_image.size
            scale = min(canvas_w / img_w, canvas_h / img_h)
            self._display_scale = scale

            new_w = int(img_w * scale)
            new_h = int(img_h * scale)

            resized = self._current_image.resize(
                (new_w, new_h), Image.Resampling.LANCZOS
            )
            self._photo_image = ImageTk.PhotoImage(resized)

            # Update canvas
            self.canvas.delete("all")
            x = canvas_w // 2
            y = canvas_h // 2
            self.canvas.create_image(x, y, image=self._photo_image, anchor="center")

            # Update frame label
            total = self.client.config.video.total_frames
            self.frame_label.config(text=f"Frame: {self.current_frame} / {total - 1}")

        except Exception as e:
            print(f"Error updating display: {e}")

    def _update_frame_info(self):
        """Update frame information display."""
        try:
            info = self.client.get_frame_info(self.current_frame)

            text = f"Frame: {info.frame_idx}\n"
            text += f"Manual: {'Yes' if info.is_manual else 'No'}\n"
            text += f"Has Masks: {'Yes' if info.has_masks else 'No'}\n"

            if info.propagated_from is not None:
                text += f"Propagated from: {info.propagated_from}\n"

            # Count points per object
            for label in self.client.config.object_labels:
                pts = info.points.get(str(label.id), [])
                if pts:
                    pos = sum(1 for p in pts if p["label"] == 1)
                    neg = len(pts) - pos
                    text += f"{label.name}: {pos}+ {neg}-\n"

            self.info_text.config(state="normal")
            self.info_text.delete("1.0", "end")
            self.info_text.insert("1.0", text)
            self.info_text.config(state="disabled")

        except Exception as e:
            print(f"Error updating info: {e}")

    def _go_to_frame(self, frame_idx: int):
        """Go to a specific frame."""
        total = self.client.config.video.total_frames
        frame_idx = max(0, min(frame_idx, total - 1))

        self.current_frame = frame_idx
        self.frame_var.set(frame_idx)
        self._update_display()
        self._update_frame_info()

    def _jump_frames(self, delta: int):
        """Jump by a number of frames."""
        self._go_to_frame(self.current_frame + delta)

    def _go_to_last_frame(self):
        """Go to the last frame."""
        self._go_to_frame(self.client.config.video.total_frames - 1)

    def _go_prev_manual(self):
        """Go to previous manually labeled frame."""
        try:
            frame = self.client.get_prev_manual(self.current_frame)
            if frame is not None:
                self._go_to_frame(frame)
            else:
                messagebox.showinfo("Info", "No previous manual frame")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _go_next_manual(self):
        """Go to next manually labeled frame."""
        try:
            frame = self.client.get_next_manual(self.current_frame)
            if frame is not None:
                self._go_to_frame(frame)
            else:
                messagebox.showinfo("Info", "No next manual frame")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _go_next_unlabeled(self):
        """Go to next frame that needs labeling."""
        try:
            frame = self.client.get_next_unlabeled(self.current_frame)
            if frame is not None:
                self._go_to_frame(frame)
            else:
                messagebox.showinfo("Info", "All frames are labeled!")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _on_undo(self):
        """Undo last point."""
        try:
            self.client.undo_point(self.current_frame)
            self._update_display()
            self._update_frame_info()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to undo: {e}")

    def _on_clear(self):
        """Clear all points on current frame."""
        if not messagebox.askyesno("Confirm", "Clear all points on this frame?"):
            return

        try:
            self.client.clear_points(self.current_frame)
            self._update_display()
            self._update_frame_info()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to clear: {e}")

    def _on_preview(self):
        """Show mask preview for current points."""
        try:
            self.status_label.config(text="Generating preview...")
            self.root.update()

            preview = self.client.get_preview(self.current_frame)
            self._current_image = preview

            # Update display with preview
            canvas_w = self.canvas.winfo_width()
            canvas_h = self.canvas.winfo_height()
            img_w, img_h = preview.size
            scale = min(canvas_w / img_w, canvas_h / img_h)

            new_w = int(img_w * scale)
            new_h = int(img_h * scale)

            resized = preview.resize((new_w, new_h), Image.Resampling.LANCZOS)
            self._photo_image = ImageTk.PhotoImage(resized)

            self.canvas.delete("all")
            x = canvas_w // 2
            y = canvas_h // 2
            self.canvas.create_image(x, y, image=self._photo_image, anchor="center")

            self.status_label.config(text="Preview generated")

        except Exception as e:
            self.status_label.config(text="Preview failed")
            messagebox.showerror("Error", f"Preview failed: {e}")

    def _on_propagate(self):
        """Propagate masks from current frame."""
        if self.is_propagating:
            return

        self.is_propagating = True
        self.propagate_btn.config(state="disabled")
        self.status_label.config(text="Propagating masks...")
        self.progress_var.set(0)
        self.root.update()

        def do_propagate():
            try:
                result = self.client.propagate(self.current_frame)

                self.root.after(0, lambda: self._on_propagate_complete(result))

            except Exception as e:
                self.root.after(0, lambda: self._on_propagate_error(e))

        thread = threading.Thread(target=do_propagate)
        thread.start()

    def _on_propagate_complete(self, result):
        """Handle propagation completion."""
        self.is_propagating = False
        self.propagate_btn.config(state="normal")
        self.progress_var.set(100)

        if result.get("success"):
            frames = result.get("propagated_frames", 0)
            frame_range = result.get("frame_range", [0, 0])
            self.status_label.config(
                text=f"Propagated {frames} frames ({frame_range[0]}-{frame_range[1]})"
            )
            self._update_display()
            self._update_frame_info()
        else:
            error = result.get("error", "Unknown error")
            self.status_label.config(text=f"Propagation failed: {error}")
            messagebox.showerror("Error", f"Propagation failed: {error}")

    def _on_propagate_error(self, error):
        """Handle propagation error."""
        self.is_propagating = False
        self.propagate_btn.config(state="normal")
        self.status_label.config(text="Propagation failed")
        messagebox.showerror("Error", f"Propagation failed: {error}")

    def _on_save(self):
        """Save current state."""
        try:
            self.client.save()
            self.status_label.config(text="Saved successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Save failed: {e}")

    def connect(self) -> bool:
        """Connect to the server."""
        self.status_label.config(text="Connecting...")
        self.root.update()

        if not self.client.connect():
            self.status_label.config(text="Connection failed")
            messagebox.showerror("Error", "Failed to connect to server")
            return False

        # Update UI with config
        config = self.client.config

        # Set slider range
        self.frame_slider.config(to=config.video.total_frames - 1)

        # Populate labels
        self._populate_labels()

        # Update status
        status = self.client.get_status()
        self.status_label.config(
            text=f"Connected - {status.manual_frames} manual, {status.generated_frames} generated"
        )

        # Load initial frame
        self._go_to_frame(0)

        return True

    def run(self):
        """Start the UI main loop."""
        if not self.connect():
            return

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.root.mainloop()

    def _on_close(self):
        """Handle window close."""
        try:
            self.client.save()
        except:
            pass
        self.root.destroy()


def run_client(server_url: str):
    """Run the labeling client."""
    ui = LabelingUI(server_url)
    ui.run()
