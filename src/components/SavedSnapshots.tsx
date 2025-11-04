import React, { useState, useEffect } from 'react';
import { Card, CardContent } from './ui/card';
import { Button } from './ui/button';
import { Badge } from './ui/badge';
import { Input } from './ui/input';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from './ui/dialog';
import { ImageWithFallback } from './figma/ImageWithFallback';
import { Snapshot, useAuth } from '../App';
import { projectId, publicAnonKey } from '../utils/supabase/info';
import { Search, Eye, Download, Trash2, Calendar, RefreshCw } from 'lucide-react';

// Utility function to safely format dates
const formatDate = (date: Date | string): string => {
  try {
    const dateObj = date instanceof Date ? date : new Date(date);
    return dateObj.toLocaleDateString();
  } catch (error) {
    return 'Invalid date';
  }
};

export function SavedSnapshots() {
  const [snapshots, setSnapshots] = useState<Snapshot[]>([]);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedSnapshot, setSelectedSnapshot] = useState<Snapshot | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState('');
  const { accessToken } = useAuth();

  const fetchSnapshots = async () => {
    if (!accessToken) return;
    
    setIsLoading(true);
    setError('');
    
    try {
      const response = await fetch(`https://${projectId}.supabase.co/functions/v1/make-server-2eb0fb94/snapshots`, {
        headers: {
          'Authorization': `Bearer ${accessToken}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch snapshots');
      }

      const data = await response.json();
      setSnapshots(data.snapshots || []);
    } catch (error) {
      console.log('Error fetching snapshots:', error);
      setError('Failed to load snapshots');
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    fetchSnapshots();
  }, [accessToken]);

  const filteredSnapshots = snapshots.filter(snapshot =>
    snapshot.name.toLowerCase().includes(searchTerm.toLowerCase())
  );

  const handleDelete = async (id: string, e: React.MouseEvent) => {
    e.stopPropagation();
    
    if (!accessToken) return;
    
    try {
      const response = await fetch(`https://${projectId}.supabase.co/functions/v1/make-server-2eb0fb94/snapshots/${id}`, {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${accessToken}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to delete snapshot');
      }

      // Remove from local state
      setSnapshots(prev => prev.filter(s => s.id !== id));
    } catch (error) {
      console.log('Error deleting snapshot:', error);
      setError('Failed to delete snapshot');
    }
  };

  const handleDownload = async (snapshot: Snapshot, e: React.MouseEvent) => {
    e.stopPropagation();
    
    if (!accessToken) return;
    
    try {
      const response = await fetch(`https://${projectId}.supabase.co/functions/v1/make-server-2eb0fb94/snapshots/${snapshot.id}/download`, {
        headers: {
          'Authorization': `Bearer ${accessToken}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to get download URL');
      }

      const data = await response.json();
      
      // Create download link
      const link = document.createElement('a');
      link.href = data.downloadUrl;
      link.download = `${snapshot.name}.jpg`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    } catch (error) {
      console.log('Error downloading snapshot:', error);
      setError('Failed to download snapshot');
    }
  };

  if (isLoading) {
    return (
      <div className="space-y-6">
        <div className="flex flex-col sm:flex-row gap-4 items-start sm:items-center justify-between">
          <div>
            <h1 className="text-3xl">Saved Snapshots</h1>
            <p className="text-muted-foreground">
              Browse and manage your robot's drawing snapshots
            </p>
          </div>
        </div>
        <Card>
          <CardContent className="flex flex-col items-center justify-center py-12">
            <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary mb-4"></div>
            <p className="text-muted-foreground">Loading snapshots...</p>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="flex flex-col sm:flex-row gap-4 items-start sm:items-center justify-between">
        <div>
          <h1 className="text-3xl">Saved Snapshots</h1>
          <p className="text-muted-foreground">
            Browse and manage your robot's drawing snapshots
          </p>
        </div>
        <div className="flex gap-2 w-full sm:w-auto">
          <div className="relative flex-1 sm:w-64">
            <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
            <Input
              placeholder="Search snapshots..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="pl-10"
            />
          </div>
          <Button
            variant="outline"
            size="sm"
            onClick={fetchSnapshots}
            disabled={isLoading}
          >
            <RefreshCw className="h-4 w-4" />
          </Button>
        </div>
      </div>

      {error && (
        <div className="bg-destructive/10 border border-destructive/20 rounded-lg p-4">
          <p className="text-destructive">{error}</p>
        </div>
      )}

      {filteredSnapshots.length === 0 ? (
        <Card>
          <CardContent className="flex flex-col items-center justify-center py-12">
            <div className="text-center space-y-2">
              <h3>No snapshots found</h3>
              <p className="text-muted-foreground">
                {searchTerm ? 'Try adjusting your search terms' : 'Upload your first drawing snapshot to get started'}
              </p>
            </div>
          </CardContent>
        </Card>
      ) : (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
          {filteredSnapshots.map((snapshot) => (
            <Card key={snapshot.id} className="group cursor-pointer hover:shadow-lg transition-shadow">
              <CardContent className="p-4">
                <div className="aspect-square mb-3 rounded-lg overflow-hidden bg-muted">
                  {snapshot.imageUrl ? (
                    <ImageWithFallback
                      src={snapshot.imageUrl}
                      alt={snapshot.name}
                      className="w-full h-full object-cover group-hover:scale-105 transition-transform duration-200"
                    />
                  ) : (
                    <div className="w-full h-full bg-muted flex items-center justify-center">
                      <p className="text-muted-foreground text-sm">No preview</p>
                    </div>
                  )}
                </div>
                
                <div className="space-y-2">
                  <h3 className="font-medium line-clamp-1">{snapshot.name}</h3>
                  
                  <div className="flex items-center gap-2 text-sm text-muted-foreground">
                    <Calendar className="h-3 w-3" />
                    <span>{formatDate(snapshot.createdAt)}</span>
                  </div>
                  
                </div>
                
                <div className="flex gap-2 mt-4">
                  <Dialog>
                    <DialogTrigger asChild>
                      <Button variant="outline" size="sm" className="flex-1">
                        <Eye className="h-3 w-3 mr-1" />
                        View
                      </Button>
                    </DialogTrigger>
                    <DialogContent className="max-w-3xl">
                      <DialogHeader>
                        <DialogTitle>{snapshot.name}</DialogTitle>
                      </DialogHeader>
                      <div className="aspect-video rounded-lg overflow-hidden bg-muted">
                        {snapshot.imageUrl ? (
                          <ImageWithFallback
                            src={snapshot.imageUrl}
                            alt={snapshot.name}
                            className="w-full h-full object-contain"
                          />
                        ) : (
                          <div className="w-full h-full bg-muted flex items-center justify-center">
                            <p className="text-muted-foreground">No preview available</p>
                          </div>
                        )}
                      </div>
                      <div className="flex justify-between items-center pt-4">
                        <div className="text-sm text-muted-foreground">
                          Created: {formatDate(snapshot.createdAt)} 
                        </div>
                        <div className="flex gap-2">
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={(e) => handleDownload(snapshot, e)}
                          >
                            <Download className="h-3 w-3 mr-1" />
                            Download
                          </Button>
                        </div>
                      </div>
                    </DialogContent>
                  </Dialog>
                  
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={(e) => handleDownload(snapshot, e)}
                  >
                    <Download className="h-3 w-3" />
                  </Button>
                  
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={(e) => handleDelete(snapshot.id, e)}
                    className="text-destructive hover:text-destructive"
                  >
                    <Trash2 className="h-3 w-3" />
                  </Button>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      )}
    </div>
  );
}