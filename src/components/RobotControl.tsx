import React, { useState, useRef, useEffect } from "react";
import {
  Card,
  CardContent,
  CardHeader,
  CardTitle,
} from "./ui/card";
import { Button } from "./ui/button";
import { Badge } from "./ui/badge";
import { Separator } from "./ui/separator";
import { Input } from "./ui/input";
import { Label } from "./ui/label";

import { Alert, AlertDescription } from "./ui/alert";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "./ui/select";
import {
  Bot,
  WifiOff,
  CheckCircle,
  Send,
  Lightbulb,
  Settings,
  Video,
  VideoOff,
  Camera
} from "lucide-react";
import { toast } from "sonner@2.0.3";

interface RobotStatus {
  isConnected: boolean;
  ledOn: boolean;
}

interface ImageInfo {
  name: string;
  filename: string;
  path: string;
}

export function RobotControl() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    isConnected: true,
    ledOn: false,
  });

  const [isConnecting, setIsConnecting] = useState(false);
  const [isSendingCommands, setIsSendingCommands] = useState(false);
  const [isCameraEnabled, setIsCameraEnabled] = useState(false);
  const [cameraError, setCameraError] = useState<string | null> (null);
  const [robotIpAddress, setRobotIpAddress] = useState<string>("172.20.10.2");
  const [availableImages, setAvailableImages] = useState<ImageInfo[]>([]);
  const [selectedImage, setSelectedImage] = useState<string>("myimage");

  const wsRef = useRef<WebSocket | null>(null);
  const videoref = useRef<HTMLImageElement | null>(null);
  const streamRef = useRef<MediaStream | null>(null);

  // Fetch available images on component mount
  useEffect(() => {
    const fetchImages = async () => {
      try {
        const response = await fetch("http://localhost:500/images");
        if (response.ok) {
          const data = await response.json();
          const images = data.images || [];
          setAvailableImages(images);
          // Set default selected image if available
          if (images.length > 0) {
            setSelectedImage((currentSelected) => {
              const imageNames = images.map((img: ImageInfo) => img.name);
              // Keep current selection if it exists, otherwise use first image
              return imageNames.includes(currentSelected) ? currentSelected : images[0].name;
            });
          }
        } else {
          console.error("Failed to fetch images");
        }
      } catch (error) {
        console.error("Error fetching images:", error);
      }
    };
    fetchImages();
  }, []);

  // Set video source when camera is enabled and image element is available
  useEffect(() => {
    if (isCameraEnabled && videoref.current) {
      console.log("Setting image source to backend MJPEG stream");
      videoref.current.src = "http://localhost:500/video_feed";
      toast.success("Robot detection camera started");
      // Add timestamp to prevent caching
      const videoUrl = `http://localhost:500/video_feed?t=${Date.now()}`;
      videoref.current.src = videoUrl;
      console.log("Video URL set to:", videoUrl);
      // Handle errors
      videoref.current.onerror = (e) => {
        console.error("Error loading video stream:", e);
        const errorMsg = "Failed to load video stream. Make sure the backend server is running at http://localhost:500";
        setCameraError(errorMsg);
        toast.error(errorMsg);
        setIsCameraEnabled(false);
      };
      
      // Handle successful load
      videoref.current.onload = () => {
        console.log("Video stream loaded successfully");
        toast.success("Robot detection camera started");
      };
    }
  }, [isCameraEnabled]);

  //clean up websocket and camera on unmount
  useEffect(() => {
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
      stopCamera();
    };
  }, []);

const startCamera = async () => {
  console.log("startCamera called");
  try {
    setCameraError(null);
    setIsCameraEnabled(true);
  } catch (error) {
    console.error("Camera error:", error);
    const errorMessage = error instanceof Error ? error.message: "Failed to access camera";
    setCameraError(errorMessage);
    toast.error("Failed to access camera");
    setIsCameraEnabled(false);
  }
};

const stopCamera = () => {
  if (streamRef.current) {
    streamRef.current.getTracks().forEach(track => track.stop());
    streamRef.current = null;
  }
  if (videoref.current) {
    videoref.current.srcObject = null;
    // Clear the image source for MJPEG stream
    videoref.current.src = "";
    videoref.current.onerror = null;
  }
  setIsCameraEnabled(false);
  setCameraError(null);
};


  // --- Connection handling ---
  const handleConnect = async () => {
    if (!robotIpAddress.trim()) {
      toast.error("Please enter a valid IP address");
      return;
    }

    setIsConnecting(true);

    try {
      const ws = new WebSocket(`ws://${robotIpAddress.trim()}/ws`);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log("WebSocket connected");
        toast.success("Connected to robot");

        ws.send("START_CONNECTION");

        setRobotStatus({
          isConnected: true,
          ledOn: true,
        });

        setIsConnecting(false);
      };

      ws.onerror = (error) => {
        console.error("WebSocket error:", error);
        toast.error("Failed to connect to robot");
        setIsConnecting(false);
      };

      ws.onclose = () => {
        console.log("WebSocket disconnected");
        setRobotStatus({
          isConnected: false,
          ledOn: false,
        });
      };

      ws.onmessage = async (event) => {
        console.log("Message from robot:", event.data);

        try {
          const response = await fetch("http://localhost:500/robot_pos");
          if (response.ok) {
            const {x, y} = await response.json();
            wsRef.current.send(`RobotPos: x=${x} y=${y}`);
          }
          else {
            console.log("error sending robot position");
          }
        } catch(e) {
          console.log("error sending robot position");
        }
      };

    } catch (error) {
      console.error("Connection error:", error);
      toast.error("Failed to connect to robot");
      setIsConnecting(false);
    }
  };

  const handleDisconnect = () => {
    if (wsRef.current) {
      if (wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send("END_CONNECTION");
      }
      wsRef.current.close();
      wsRef.current = null;
    }
    stopCamera();
    setRobotStatus({
      isConnected: false,
      ledOn: false,
    });
  };

  const handleSendCommands = async () => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      toast.error("Robot not connected");
      return;
    }

    if (!selectedImage) {
      toast.error("Please select an image");
      return;
    }

    setIsSendingCommands(true);

    try {
      // Load the file from public/ using selected image name
      const response_stepper = await fetch(`backend/color_output/${selectedImage}1.gcode`);
      const stepperGCodeText = await response_stepper.text();
      const response_gear = await fetch(`backend/color_output/${selectedImage}2.gcode`);
      const gearGCodeText = await response_gear.text();

      const stepper_contours = stepperGCodeText
        .split(/(?=Contour\s+\d+)/) // split BEFORE each "Contour <num>"
        .map(chunk => chunk.trim()) // clean up whitespace
        .filter(chunk => chunk.length > 0); // remove empties

      console.log(`Found ${stepper_contours.length} stepper contours.`);

      // Example: send one contour at a time through a WebSocket
      for (const contour of stepper_contours) {
        console.log("Sending contour chunk:\n", contour);
        wsRef.current.send(contour+'\n');
        await new Promise(resolve => setTimeout(resolve, 10));
      }
      wsRef.current.send("STEPPER_GCODE_END"); // sent all gcode

      toast.success(`Sent ${stepper_contours.length} stepper contours`);

      const gear_contours = gearGCodeText
        .split(/(?=Contour\s+\d+)/) // split BEFORE each "Contour <num>"
        .map(chunk => chunk.trim()) // clean up whitespace
        .filter(chunk => chunk.length > 0); // remove empties

      console.log(`Found ${gear_contours.length} gear contours.`);

      // Example: send one contour at a time through a WebSocket
      for (const contour of gear_contours) {
        console.log("Sending contour chunk:\n", contour);
        wsRef.current.send(contour+'\n');
        await new Promise(resolve => setTimeout(resolve, 10));
      }
      wsRef.current.send("GEAR_GCODE_END"); // sent all gcode

      toast.success(`Sent ${gear_contours.length} gear contours`);
    } catch (err) {
      console.error("Failed to load or send GCode:", err);
      toast.error("Error loading GCode file");
    } finally {
      setIsSendingCommands(false);
    }
  };

  return (
    <div className="space-y-6">
      <div>
        <h1 className="text-3xl">Robot Control</h1>
        <p className="text-muted-foreground">
          Connect and control your DrawBot whiteboard robot
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
         {/* Webcam Feed - Full width when camera is enabled */}
          {robotStatus.isConnected && isCameraEnabled && (
            <Card className="lg:col-span-3">
              <CardHeader>
                <div className="flex items-center justify-between">
                  <CardTitle className="flex items-center gap-2">
                    <Camera className="h-5 w-5" />
                    Robot Camera Feed
                  </CardTitle>
                  <Button 
                    variant="outline" 
                    size="sm"
                    onClick={stopCamera}
                  >
                    <VideoOff className="h-4 w-4 mr-2" />
                    Stop Camera
                  </Button>
                </div>
              </CardHeader>
              <CardContent>
                <div className="relative w-full bg-black rounded-lg overflow-hidden" style={{ aspectRatio: '16/9' }}>
                  <img
                    ref={videoref}
                    className="w-full h-full object-contain"
                    alt="Robot camera feed with ArUco detection"
                    style={{ display: 'block' }}
                  />
                </div>
                <p className="text-xs text-muted-foreground mt-2">
                  Live video feed with ArUco marker detection from robot camera.
                </p>
              </CardContent>
            </Card>
          )}
        {/* Connection Panel */}
        <Card className="lg:col-span-1">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Bot className="h-5 w-5" />
              Robot Connection
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            {!robotStatus.isConnected ? (
              <div className="space-y-4">
                <div className="text-center py-4">
                  <WifiOff className="h-8 w-8 mx-auto text-muted-foreground mb-2" />
                  <p className="text-sm text-muted-foreground">
                    No robot connected
                  </p>
                </div>

                {/* LED Status - OFF */}
                <div className="flex items-center justify-center gap-3 p-4 bg-muted rounded-lg">
                  <Lightbulb className="h-6 w-6 text-muted-foreground" />
                  <span className="text-sm">LED: OFF</span>
                </div>

                <div className="space-y-2">
                  <h4 className="font-medium">Robot Connection</h4>
                  <div className="space-y-2">
                    <Label htmlFor="robot-ip">Robot IP Address</Label>
                    <Input
                      id="robot-ip"
                      type="text"
                      placeholder="172.20.10.2"
                      value={robotIpAddress}
                      onChange={(e) => setRobotIpAddress(e.target.value)}
                      disabled={isConnecting || robotStatus.isConnected}
                      className="w-full"
                    />
                  </div>
                  <Button
                    className="w-full"
                    onClick={handleConnect}
                    disabled={isConnecting || !robotIpAddress.trim()}
                  >
                    {isConnecting ? "Connecting..." : "Connect"}
                  </Button>
                </div>
              </div>
            ) : (
              <div className="space-y-4">
                <div className="text-center py-4">
                  <div className="flex items-center justify-center gap-2 mb-2">
                    <CheckCircle className="h-8 w-8 text-green-500" />
                  </div>
                  <p className="font-medium">DrawBot-001</p>
                  <Badge
                    variant="outline"
                    className="text-green-600 border-green-600"
                  >
                    Connected
                  </Badge>
                  <p className="text-sm text-muted-foreground mt-2">
                    IP: {robotIpAddress}
                  </p>
                </div>

                {/* LED Status - ON */}
                <div className="flex items-center justify-center gap-3 p-4 bg-green-50 dark:bg-green-950 rounded-lg">
                  <Lightbulb className="h-6 w-6 text-green-600 fill-green-600" />
                  <span className="text-sm font-medium text-green-600">
                    LED: ON
                  </span>
                </div>

                <Button
                  variant="outline"
                  onClick={handleDisconnect}
                  className="w-full"
                >
                  Disconnect
                </Button>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Control Panel */}
        <Card className="lg:col-span-2">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Send className="h-5 w-5" />
              Robot Controls
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            {!robotStatus.isConnected ? (
              <div className="text-center py-12">
                <Bot className="h-12 w-12 mx-auto text-muted-foreground mb-4" />
                <p className="text-muted-foreground">
                  Connect to the robot to access controls
                </p>
              </div>
            ) : (
              <>
                {/* Image Selection */}
                <div className="space-y-4">
                  <h4 className="font-medium">Select Image</h4>
                  <div className="space-y-2">
                    <Label htmlFor="image-select">Choose an image to send</Label>
                    <Select
                      value={selectedImage}
                      onValueChange={setSelectedImage}
                    >
                      <SelectTrigger id="image-select" className="w-full">
                        <SelectValue placeholder="Select an image" />
                      </SelectTrigger>
                      <SelectContent>
                        {availableImages.length === 0 ? (
                          <SelectItem value="none" disabled>
                            No images available
                          </SelectItem>
                        ) : (
                          availableImages.map((image) => (
                            <SelectItem key={image.name} value={image.name}>
                              {image.name}
                            </SelectItem>
                          ))
                        )}
                      </SelectContent>
                    </Select>
                    {availableImages.length === 0 && (
                      <p className="text-sm text-muted-foreground">
                        No processed images found. Upload and process an image first.
                      </p>
                    )}
                  </div>
                </div>

                {/* Processed Image Preview */}
                {selectedImage && (
                  <div className="space-y-4">
                    <h4 className="font-medium">Processed Image Preview</h4>
                    <div className="border rounded-lg overflow-hidden bg-muted">
                      <img
                        src={`backend/color_output/${selectedImage}.png`}
                        alt="Processed drawing"
                        className="w-full h-auto"
                        onError={(e) => {
                          console.error("Failed to load image:", selectedImage);
                          toast.error(`Failed to load image: ${selectedImage}`);
                        }}
                      />
                    </div>
                  </div>
                )}

                {/* Send Commands */}
                <div className="space-y-4">
                  <h4 className="font-medium">GCode Commands</h4>
                  <p className="text-sm text-muted-foreground">
                    Send the processed GCode file to the robot
                  </p>

                  <Button
                    onClick={handleSendCommands}
                    disabled={isSendingCommands}
                    size="lg"
                    className="w-full"
                  >
                    {isSendingCommands ? (
                      <span className="animate-pulse">Sending Commands...</span>
                    ) : (
                      <>
                        <Send className="h-5 w-5 mr-2" />
                        Send Commands
                      </>
                    )}
                  </Button>
                </div>

                {/* Camera Control */}
                <div className="space-y-4">
                  <h4 className="font-medium">
                    Camera Feed
                  </h4>
                  {!isCameraEnabled ? (
                    <div className="space-y-3">
                      <p className="text-sm text-muted-foreground">
                        Enable camera to monitor robot movements in real-time
                      </p>
                      {cameraError && (
                        <div className="text-sm text-red-500 p-3 bg-red-50 rounded-md">
                          <p className="font-medium">Camera Error:</p>
                          <p>{cameraError}</p>
                          <p className="text-xs mt-2">Make sure camera permissions are granted in your browser settings.</p>
                        </div>
                      )}
                      <Button 
                        onClick={startCamera}
                        variant="outline"
                        className="w-full flex items-center justify-center gap-2"
                      >
                        <Video className="h-4 w-4" />
                        Start Camera
                      </Button>
                    </div>
                  ) : (
                    <div className="text-sm text-green-600 p-3 bg-green-50 rounded-md flex items-center gap-2">
                      <CheckCircle className="h-4 w-4" />
                      <span>Camera is active (see above for video feed)</span>
                    </div>
                  )}
                </div>
                <Separator />
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
