"""REST API server for video labeling."""

import io
import json
import traceback
from typing import Dict, Optional

import cv2
import numpy as np
from flask import Flask, jsonify, request, send_file, Response
from flask_cors import CORS
from tqdm import tqdm

from .annotation_manager import AnnotationManager
from .config import ServerConfig
from .sam3_tracker import SAM3Tracker
from .video_handler import VideoHandler


def create_app(config: ServerConfig) -> Flask:
    """Create and configure the Flask application."""

    app = Flask(__name__)
    CORS(app)  # Enable CORS for client access

    # Ensure directories exist
    config.ensure_directories()

    # Initialize components
    video_handler = VideoHandler(config.video_path)

    annotation_manager = AnnotationManager(
        manual_dir=config.manual_annotations_dir,
        generated_dir=config.generated_annotations_dir,
        video_width=video_handler.width,
        video_height=video_handler.height,
        object_labels=[{"id": l.id, "name": l.name} for l in config.object_labels],
        output_width=config.output_width,
    )

    # Try to load existing state
    if annotation_manager.load_state():
        print("Loaded existing annotation state")
    else:
        print("Starting with fresh annotation state")

    # Initialize SAM3 tracker (lazy loading)
    tracker: Optional[SAM3Tracker] = None

    def get_tracker() -> SAM3Tracker:
        nonlocal tracker
        if tracker is None:
            tracker = SAM3Tracker(
                gpu_id=config.gpu_id,
                inference_width=config.inference_width,
            )
            tracker.set_video(config.video_path)
        return tracker

    # Build color map from config
    color_map = {label.id: tuple(label.color) for label in config.object_labels}

    # ==================== API Routes ====================

    @app.route("/api/config", methods=["GET"])
    def get_config():
        """Get server configuration."""
        return jsonify(
            {
                **config.to_dict(),
                "video": video_handler.get_metadata(),
            }
        )

    @app.route("/api/status", methods=["GET"])
    def get_status():
        """Get annotation status."""
        return jsonify(annotation_manager.get_status())

    @app.route("/api/frame/<int:frame_idx>", methods=["GET"])
    def get_frame(frame_idx: int):
        """Get a frame image."""
        include_masks = request.args.get("masks", "false").lower() == "true"
        include_points = request.args.get("points", "false").lower() == "true"
        quality = int(request.args.get("quality", 90))

        frame = video_handler.get_frame(frame_idx)
        if frame is None:
            return jsonify({"error": "Frame not found"}), 404

        # Overlay masks if requested
        if include_masks:
            masks = annotation_manager.get_all_masks(frame_idx)
            if masks:
                frame = video_handler.overlay_masks(
                    frame, masks, color_map, config.mask_alpha
                )

        # Draw points if requested
        if include_points:
            for label in config.object_labels:
                points = annotation_manager.get_points(frame_idx, label.id)
                if points:
                    pts = [(p.x, p.y) for p in points]
                    labels = [p.label for p in points]
                    frame = video_handler.draw_points(
                        frame, pts, labels, label.id, tuple(label.color)
                    )

        # Encode as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, buffer = cv2.imencode(".jpg", frame, encode_param)

        return send_file(
            io.BytesIO(buffer.tobytes()),
            mimetype="image/jpeg",
        )

    @app.route("/api/frame/<int:frame_idx>/info", methods=["GET"])
    def get_frame_info(frame_idx: int):
        """Get information about a frame."""
        ann = annotation_manager.annotations.get(frame_idx)

        points_by_obj = {}
        if ann:
            for label in config.object_labels:
                obj_points = [p for p in ann.points if p.obj_id == label.id]
                if obj_points:
                    points_by_obj[label.id] = [
                        {"x": p.x, "y": p.y, "label": p.label} for p in obj_points
                    ]

        return jsonify(
            {
                "frame_idx": frame_idx,
                "is_manual": ann.is_manual if ann else False,
                "has_masks": annotation_manager.has_masks(frame_idx),
                "propagated_from": ann.propagated_from if ann else None,
                "points": points_by_obj,
                "mask_object_ids": list(ann.masks.keys()) if ann else [],
            }
        )

    @app.route("/api/points", methods=["POST"])
    def add_point():
        """Add a point prompt."""
        data = request.json

        frame_idx = data["frame_idx"]
        x = data["x"]
        y = data["y"]
        label = data["label"]  # 1 or 0
        obj_id = data["obj_id"]

        ann = annotation_manager.add_point(frame_idx, x, y, label, obj_id)

        # Clear propagated masks from this frame onwards
        annotation_manager.clear_propagated_masks(frame_idx)

        # Save state
        annotation_manager.save_state()

        return jsonify(
            {
                "success": True,
                "frame_idx": frame_idx,
                "points_count": len(ann.points),
            }
        )

    @app.route("/api/points/undo", methods=["POST"])
    def undo_point():
        """Undo the last point on a frame."""
        data = request.json
        frame_idx = data["frame_idx"]

        ann = annotation_manager.undo_last_point(frame_idx)

        if ann:
            annotation_manager.clear_propagated_masks(frame_idx)
            annotation_manager.save_state()
            return jsonify(
                {
                    "success": True,
                    "points_count": len(ann.points),
                }
            )

        return jsonify({"success": False, "error": "No points to undo"})

    @app.route("/api/points/clear", methods=["POST"])
    def clear_points():
        """Clear points from a frame."""
        data = request.json
        frame_idx = data["frame_idx"]
        obj_id = data.get("obj_id")  # Optional: clear only specific object

        ann = annotation_manager.clear_points(frame_idx, obj_id)

        annotation_manager.clear_propagated_masks(frame_idx)
        annotation_manager.save_state()

        return jsonify(
            {
                "success": True,
                "points_count": len(ann.points) if ann else 0,
            }
        )

    @app.route("/api/points/delete", methods=["POST"])
    def delete_point():
        """Delete a specific point by index."""
        data = request.json
        frame_idx = data["frame_idx"]
        point_index = data["point_index"]

        ann = annotation_manager.remove_point(frame_idx, point_index)

        if ann is not None:
            annotation_manager.clear_propagated_masks(frame_idx)
            annotation_manager.save_state()
            return jsonify(
                {
                    "success": True,
                    "points_count": len(ann.points),
                }
            )

        return jsonify({"success": False, "error": "Point not found"})

    @app.route("/api/preview", methods=["POST"])
    def preview_mask():
        """Get a preview of the mask for current points."""
        data = request.json
        frame_idx = data["frame_idx"]
        quality = data.get("quality", 85)

        t = get_tracker()

        # Collect all points for all objects
        points_by_obj = {}
        for label in config.object_labels:
            points, labels = annotation_manager.get_points_for_sam(frame_idx, label.id)
            if len(points) > 0:
                points_by_obj[label.id] = (points, labels)

        # Get preview masks for all objects at once
        all_masks = {}
        if points_by_obj:
            all_masks = t.get_multi_object_preview(frame_idx, points_by_obj)

        # Get frame and overlay masks
        frame = video_handler.get_frame(frame_idx)
        if all_masks:
            frame = video_handler.overlay_masks(
                frame, all_masks, color_map, config.mask_alpha
            )

        # Draw points
        for label in config.object_labels:
            points = annotation_manager.get_points(frame_idx, label.id)
            if points:
                pts = [(p.x, p.y) for p in points]
                lbls = [p.label for p in points]
                frame = video_handler.draw_points(
                    frame, pts, lbls, label.id, tuple(label.color)
                )

        # Encode as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, buffer = cv2.imencode(".jpg", frame, encode_param)

        return send_file(
            io.BytesIO(buffer.tobytes()),
            mimetype="image/jpeg",
        )

    @app.route("/api/propagate", methods=["POST"])
    def propagate_masks():
        """Propagate masks from a frame."""
        data = request.json
        frame_idx = data.get("frame_idx", 0)
        length = data.get("length", config.propagate_length)

        try:
            t = get_tracker()

            # Collect points for all objects
            points_by_obj = {}
            for label in config.object_labels:
                points, labels = annotation_manager.get_points_for_sam(
                    frame_idx, label.id
                )
                if len(points) > 0:
                    points_by_obj[label.id] = (points, labels)

            if not points_by_obj:
                return jsonify(
                    {
                        "success": False,
                        "error": "No points to propagate from",
                    }
                )

            # Propagate
            segments = t.propagate_from_frame(
                source_frame=frame_idx,
                points_by_obj=points_by_obj,
                propagate_length=length,
            )

            # Save masks and add to COCO
            for fid, masks in tqdm(
                segments.items(),
                desc="Saving masks",
                unit="frame",
            ):
                for obj_id, mask in masks.items():
                    annotation_manager.set_mask(
                        fid, obj_id, mask, propagated_from=frame_idx
                    )

                # Add to COCO format
                frame = video_handler.get_frame(fid)
                if frame is not None:
                    annotation_manager.add_to_coco(fid, frame, masks)

            print("Saving state...")
            annotation_manager.save_state()

            return jsonify(
                {
                    "success": True,
                    "propagated_frames": len(segments),
                    "frame_range": [min(segments.keys()), max(segments.keys())],
                }
            )

        except Exception as e:
            traceback.print_exc()
            return jsonify(
                {
                    "success": False,
                    "error": str(e),
                }
            ), 500

    @app.route("/api/navigation/next-unlabeled", methods=["GET"])
    def get_next_unlabeled():
        """Get the next frame that needs labeling."""
        current = int(request.args.get("current", 0))

        next_frame = annotation_manager.get_next_unlabeled_frame(
            current, video_handler.total_frames
        )

        return jsonify(
            {
                "frame_idx": next_frame,
                "found": next_frame is not None,
            }
        )

    @app.route("/api/navigation/manual-frames", methods=["GET"])
    def get_manual_frames():
        """Get list of manually labeled frames."""
        return jsonify(
            {
                "frames": annotation_manager.get_manual_frames(),
            }
        )

    @app.route("/api/navigation/prev-manual", methods=["GET"])
    def get_prev_manual():
        """Get previous manually labeled frame."""
        current = int(request.args.get("current", 0))

        prev_frame = annotation_manager.get_prev_manual_frame(current)

        return jsonify(
            {
                "frame_idx": prev_frame,
                "found": prev_frame is not None,
            }
        )

    @app.route("/api/navigation/next-manual", methods=["GET"])
    def get_next_manual():
        """Get next manually labeled frame."""
        current = int(request.args.get("current", 0))

        next_frame = annotation_manager.get_next_manual_frame(current)

        return jsonify(
            {
                "frame_idx": next_frame,
                "found": next_frame is not None,
            }
        )

    @app.route("/api/save", methods=["POST"])
    def save_state():
        """Force save current state."""
        annotation_manager.save_state()
        return jsonify({"success": True})

    @app.route("/api/export/coco", methods=["GET"])
    def export_coco():
        """Export annotations in COCO format."""
        return jsonify(annotation_manager.coco)

    @app.teardown_appcontext
    def cleanup(exception=None):
        """Clean up on app shutdown."""
        annotation_manager.save_state()
        if tracker:
            tracker.cleanup()

    return app


def run_server(config_path: str):
    """Run the labeling server."""
    config = ServerConfig.from_yaml(config_path)
    app = create_app(config)

    print(f"\n{'=' * 60}")
    print(f"Video Labeling Server")
    print(f"{'=' * 60}")
    print(f"Video: {config.video_path}")
    print(f"Objects: {[l.name for l in config.object_labels]}")
    print(f"Propagate length: {config.propagate_length} frames")
    print(f"Manual annotations: {config.manual_annotations_dir}")
    print(f"Generated annotations: {config.generated_annotations_dir}")
    print(f"{'=' * 60}")
    print(f"Server running at: http://{config.host}:{config.port}")
    print(f"{'=' * 60}\n")

    app.run(host=config.host, port=config.port, threaded=True)
