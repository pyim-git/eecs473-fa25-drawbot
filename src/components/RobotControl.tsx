import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from './ui/card';
import { Button } from './ui/button';
import { Badge } from './ui/badge';
import { Progress } from './ui/progress';
import { Alert, AlertDescription } from './ui/alert';
import { Separator } from './ui/separator';
import { 
  Bot, 
  Wifi, 
  WifiOff, 
  Battery, 
  Play, 
  Pause, 
  Square, 
  RotateCcw,
  Settings,
  AlertTriangle,
  CheckCircle,
  Zap
} from 'lucide-react';

interface RobotStatus {
  isConnected: boolean;
  batteryLevel: number;
  isDrawing: boolean;
  currentTask: string | null;
  position: { x: number; y: number };
  drawingProgress: number;
}

export function RobotControl() {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    isConnected: false,
    batteryLevel: 0,
    isDrawing: false,
    currentTask: null,
    position: { x: 0, y: 0 },
    drawingProgress: 0
  });

  const [isConnecting, setIsConnecting] = useState(false);
  const [availableRobots, setAvailableRobots] = useState([
    { id: 'db-001', name: 'DrawBot-001', signal: 85 },
    { id: 'db-002', name: 'DrawBot-002', signal: 92 },
  ]);

  // Simulate robot status updates
  useEffect(() => {
    if (robotStatus.isConnected) {
      const interval = setInterval(() => {
        setRobotStatus(prev => ({
          ...prev,
          batteryLevel: Math.max(0, prev.batteryLevel - 0.1),
          position: {
            x: prev.position.x + (Math.random() - 0.5) * 2,
            y: prev.position.y + (Math.random() - 0.5) * 2
          },
          drawingProgress: prev.isDrawing 
            ? Math.min(100, prev.drawingProgress + 1)
            : prev.drawingProgress
        }));
      }, 1000);

      return () => clearInterval(interval);
    }
  }, [robotStatus.isConnected, robotStatus.isDrawing]);

  const handleConnect = async (robotId: string) => {
    setIsConnecting(true);
    
    // Simulate connection process
    await new Promise(resolve => setTimeout(resolve, 2000));
    
    setRobotStatus({
      isConnected: true,
      batteryLevel: 85,
      isDrawing: false,
      currentTask: null,
      position: { x: 0, y: 0 },
      drawingProgress: 0
    });
    
    setIsConnecting(false);
  };

  const handleDisconnect = () => {
    setRobotStatus(prev => ({
      ...prev,
      isConnected: false,
      isDrawing: false,
      currentTask: null,
      drawingProgress: 0
    }));
  };

  const handleStartDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: true,
      currentTask: 'Drawing geometric pattern',
      drawingProgress: 0
    }));
  };

  const handlePauseDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: false,
      currentTask: 'Paused'
    }));
  };

  const handleStopDrawing = () => {
    setRobotStatus(prev => ({
      ...prev,
      isDrawing: false,
      currentTask: null,
      drawingProgress: 0
    }));
  };

  const handleHomePosition = () => {
    setRobotStatus(prev => ({
      ...prev,
      position: { x: 0, y: 0 },
      currentTask: 'Returning to home position'
    }));
    
    setTimeout(() => {
      setRobotStatus(prev => ({
        ...prev,
        currentTask: null
      }));
    }, 2000);
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
                  <p className="text-sm text-muted-foreground">No robot connected</p>
                </div>
                
                <div className="space-y-2">
                  <h4 className="font-medium">Available Robots</h4>
                  {availableRobots.map((robot) => (
                    <div key={robot.id} className="flex items-center justify-between p-3 border rounded-lg">
                      <div>
                        <p className="font-medium">{robot.name}</p>
                        <div className="flex items-center gap-1 text-sm text-muted-foreground">
                          <Wifi className="h-3 w-3" />
                          <span>{robot.signal}%</span>
                        </div>
                      </div>
                      <Button
                        size="sm"
                        onClick={() => handleConnect(robot.id)}
                        disabled={isConnecting}
                      >
                        {isConnecting ? 'Connecting...' : 'Connect'}
                      </Button>
                    </div>
                  ))}
                </div>
              </div>
            ) : (
              <div className="space-y-4">
                <div className="text-center py-4">
                  <div className="flex items-center justify-center gap-2 mb-2">
                    <CheckCircle className="h-8 w-8 text-green-500" />
                  </div>
                  <p className="font-medium">DrawBot-001</p>
                  <Badge variant="outline" className="text-green-600 border-green-600">
                    Connected
                  </Badge>
                </div>

                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Battery className="h-4 w-4" />
                      <span className="text-sm">Battery</span>
                    </div>
                    <span className="text-sm font-medium">{robotStatus.batteryLevel.toFixed(0)}%</span>
                  </div>
                  <Progress value={robotStatus.batteryLevel} />
                  
                  {robotStatus.batteryLevel < 20 && (
                    <Alert>
                      <AlertTriangle className="h-4 w-4" />
                      <AlertDescription>
                        Low battery! Consider charging soon.
                      </AlertDescription>
                    </Alert>
                  )}
                </div>

                <Button variant="outline" onClick={handleDisconnect} className="w-full">
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
              <Settings className="h-5 w-5" />
              Robot Controls
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-6">
            {!robotStatus.isConnected ? (
              <div className="text-center py-12">
                <Bot className="h-12 w-12 mx-auto text-muted-foreground mb-4" />
                <p className="text-muted-foreground">Connect to a robot to access controls</p>
              </div>
            ) : (
              <>
                {/* Current Status */}
                <div className="space-y-3">
                  <h4 className="font-medium">Current Status</h4>
                  <div className="grid grid-cols-2 gap-4">
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Status</p>
                      <div className="flex items-center gap-2">
                        {robotStatus.isDrawing ? (
                          <>
                            <Zap className="h-4 w-4 text-green-500" />
                            <span className="text-green-600">Drawing</span>
                          </>
                        ) : (
                          <>
                            <div className="h-4 w-4 rounded-full bg-muted-foreground"></div>
                            <span>Idle</span>
                          </>
                        )}
                      </div>
                    </div>
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Position</p>
                      <p className="font-mono text-sm">
                        X: {robotStatus.position.x.toFixed(1)} Y: {robotStatus.position.y.toFixed(1)}
                      </p>
                    </div>
                  </div>
                  
                  {robotStatus.currentTask && (
                    <div className="space-y-1">
                      <p className="text-sm text-muted-foreground">Current Task</p>
                      <p className="text-sm">{robotStatus.currentTask}</p>
                    </div>
                  )}
                </div>

                <Separator />

                {/* Drawing Controls */}
                <div className="space-y-4">
                  <h4 className="font-medium">Drawing Controls</h4>
                  
                  {robotStatus.isDrawing && (
                    <div className="space-y-2">
                      <div className="flex items-center justify-between text-sm">
                        <span>Progress</span>
                        <span>{robotStatus.drawingProgress.toFixed(0)}%</span>
                      </div>
                      <Progress value={robotStatus.drawingProgress} />
                    </div>
                  )}
                  
                  <div className="flex gap-2">
                    {!robotStatus.isDrawing ? (
                      <Button onClick={handleStartDrawing} className="flex items-center gap-2">
                        <Play className="h-4 w-4" />
                        Start Drawing
                      </Button>
                    ) : (
                      <Button onClick={handlePauseDrawing} variant="outline" className="flex items-center gap-2">
                        <Pause className="h-4 w-4" />
                        Pause
                      </Button>
                    )}
                    
                    <Button
                      onClick={handleStopDrawing}
                      variant="outline"
                      disabled={!robotStatus.isDrawing && robotStatus.drawingProgress === 0}
                      className="flex items-center gap-2"
                    >
                      <Square className="h-4 w-4" />
                      Stop
                    </Button>
                    
                    <Button
                      onClick={handleHomePosition}
                      variant="outline"
                      disabled={robotStatus.isDrawing}
                      className="flex items-center gap-2"
                    >
                      <RotateCcw className="h-4 w-4" />
                      Home
                    </Button>
                  </div>
                </div>

                <Separator />

                {/* Quick Actions */}
                <div className="space-y-4">
                  <h4 className="font-medium">Quick Actions</h4>
                  <div className="grid grid-cols-2 gap-2">
                    <Button variant="outline" size="sm" disabled>
                      Load Snapshot
                    </Button>
                    <Button variant="outline" size="sm" disabled>
                      Custom Path
                    </Button>
                    <Button variant="outline" size="sm" disabled>
                      Calibrate
                    </Button>
                    <Button variant="outline" size="sm" disabled>
                      Test Pattern
                    </Button>
                  </div>
                  <p className="text-xs text-muted-foreground">
                    Advanced features coming soon
                  </p>
                </div>
              </>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}