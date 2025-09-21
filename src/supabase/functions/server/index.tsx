import { Hono } from "npm:hono";
import { cors } from "npm:hono/cors";
import { logger } from "npm:hono/logger";
import { createClient } from "npm:@supabase/supabase-js@2";
import * as kv from "./kv_store.tsx";
const app = new Hono();

// Create Supabase client
const supabase = createClient(
  Deno.env.get('SUPABASE_URL')!,
  Deno.env.get('SUPABASE_SERVICE_ROLE_KEY')!,
);

// Storage bucket name
const bucketName = 'make-2eb0fb94-snapshots';

// Enable logger
app.use('*', logger(console.log));

// Enable CORS for all routes and methods
app.use(
  "/*",
  cors({
    origin: "*",
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    exposeHeaders: ["Content-Length"],
    maxAge: 600,
  }),
);

// Initialize storage bucket on startup
const initializeBucket = async () => {
  try {
    const { data: buckets } = await supabase.storage.listBuckets();
    const bucketExists = buckets?.some(bucket => bucket.name === bucketName);
    if (!bucketExists) {
      const { error } = await supabase.storage.createBucket(bucketName, {
        public: false,
        fileSizeLimit: 10485760, // 10MB
        allowedMimeTypes: ['image/*']
      });
      if (error) {
        console.log(`Error creating bucket: ${error.message}`);
      } else {
        console.log(`Created storage bucket: ${bucketName}`);
      }
    }
  } catch (error) {
    console.log(`Error initializing bucket: ${error}`);
  }
};

// Initialize bucket
initializeBucket();

// Health check endpoint
app.get("/make-server-2eb0fb94/health", (c) => {
  return c.json({ status: "ok" });
});

// Auth endpoints
app.post("/make-server-2eb0fb94/auth/signup", async (c) => {
  try {
    const { email, password, name } = await c.req.json();
    
    const { data, error } = await supabase.auth.admin.createUser({
      email,
      password,
      user_metadata: { name },
      // Automatically confirm the user's email since an email server hasn't been configured.
      email_confirm: true
    });
    
    if (error) {
      return c.json({ error: `Signup error: ${error.message}` }, 400);
    }
    
    return c.json({ user: data.user });
  } catch (error) {
    console.log(`Signup error: ${error}`);
    return c.json({ error: "Internal server error during signup" }, 500);
  }
});

// Get user snapshots
app.get("/make-server-2eb0fb94/snapshots", async (c) => {
  try {
    const accessToken = c.req.header('Authorization')?.split(' ')[1];
    const { data: { user }, error: authError } = await supabase.auth.getUser(accessToken);
    
    if (!user?.id) {
      return c.json({ error: "Unauthorized" }, 401);
    }
    
    const snapshotsKey = `user:${user.id}:snapshots`;
    const snapshots = await kv.get(snapshotsKey) || [];
    
    // Generate signed URLs for images
    const snapshotsWithUrls = await Promise.all(
      snapshots.map(async (snapshot: any) => {
        if (snapshot.storagePath) {
          const { data: signedUrl } = await supabase.storage
            .from(bucketName)
            .createSignedUrl(snapshot.storagePath, 3600); // 1 hour expiry
          
          return {
            ...snapshot,
            imageUrl: signedUrl?.signedUrl || snapshot.imageUrl
          };
        }
        return snapshot;
      })
    );
    
    return c.json({ snapshots: snapshotsWithUrls });
  } catch (error) {
    console.log(`Error fetching snapshots: ${error}`);
    return c.json({ error: "Error fetching snapshots" }, 500);
  }
});

// Upload snapshot
app.post("/make-server-2eb0fb94/snapshots/upload", async (c) => {
  try {
    const accessToken = c.req.header('Authorization')?.split(' ')[1];
    const { data: { user }, error: authError } = await supabase.auth.getUser(accessToken);
    
    if (!user?.id) {
      return c.json({ error: "Unauthorized" }, 401);
    }
    
    const formData = await c.req.formData();
    const file = formData.get('file') as File;
    const name = formData.get('name') as string;
    const canvasSize = formData.get('canvasSize') as string;
    
    if (!file || !name || !canvasSize) {
      return c.json({ error: "Missing required fields" }, 400);
    }
    
    // Generate unique file path
    const fileExtension = file.name.split('.').pop();
    const fileName = `${user.id}/${Date.now()}-${Math.random().toString(36).substring(7)}.${fileExtension}`;
    
    // Upload to Supabase Storage
    const { data: uploadData, error: uploadError } = await supabase.storage
      .from(bucketName)
      .upload(fileName, file);
    
    if (uploadError) {
      console.log(`Upload error: ${uploadError.message}`);
      return c.json({ error: `Upload failed: ${uploadError.message}` }, 400);
    }
    
    // Create snapshot metadata
    const snapshot = {
      id: Date.now().toString(),
      name,
      createdAt: new Date().toISOString(),
      canvasSize,
      storagePath: fileName,
      fileSize: file.size
    };
    
    // Save to KV store
    const snapshotsKey = `user:${user.id}:snapshots`;
    const existingSnapshots = await kv.get(snapshotsKey) || [];
    const updatedSnapshots = [snapshot, ...existingSnapshots];
    await kv.set(snapshotsKey, updatedSnapshots);
    
    return c.json({ snapshot });
  } catch (error) {
    console.log(`Error uploading snapshot: ${error}`);
    return c.json({ error: "Error uploading snapshot" }, 500);
  }
});

// Delete snapshot
app.delete("/make-server-2eb0fb94/snapshots/:id", async (c) => {
  try {
    const accessToken = c.req.header('Authorization')?.split(' ')[1];
    const { data: { user }, error: authError } = await supabase.auth.getUser(accessToken);
    
    if (!user?.id) {
      return c.json({ error: "Unauthorized" }, 401);
    }
    
    const snapshotId = c.req.param('id');
    const snapshotsKey = `user:${user.id}:snapshots`;
    const snapshots = await kv.get(snapshotsKey) || [];
    
    // Find snapshot to delete
    const snapshotToDelete = snapshots.find((s: any) => s.id === snapshotId);
    if (!snapshotToDelete) {
      return c.json({ error: "Snapshot not found" }, 404);
    }
    
    // Delete from storage if it has a storage path
    if (snapshotToDelete.storagePath) {
      await supabase.storage
        .from(bucketName)
        .remove([snapshotToDelete.storagePath]);
    }
    
    // Remove from KV store
    const updatedSnapshots = snapshots.filter((s: any) => s.id !== snapshotId);
    await kv.set(snapshotsKey, updatedSnapshots);
    
    return c.json({ success: true });
  } catch (error) {
    console.log(`Error deleting snapshot: ${error}`);
    return c.json({ error: "Error deleting snapshot" }, 500);
  }
});

// Download snapshot
app.get("/make-server-2eb0fb94/snapshots/:id/download", async (c) => {
  try {
    const accessToken = c.req.header('Authorization')?.split(' ')[1];
    const { data: { user }, error: authError } = await supabase.auth.getUser(accessToken);
    
    if (!user?.id) {
      return c.json({ error: "Unauthorized" }, 401);
    }
    
    const snapshotId = c.req.param('id');
    const snapshotsKey = `user:${user.id}:snapshots`;
    const snapshots = await kv.get(snapshotsKey) || [];
    
    const snapshot = snapshots.find((s: any) => s.id === snapshotId);
    if (!snapshot || !snapshot.storagePath) {
      return c.json({ error: "Snapshot not found" }, 404);
    }
    
    // Create signed URL for download
    const { data: signedUrl, error } = await supabase.storage
      .from(bucketName)
      .createSignedUrl(snapshot.storagePath, 300); // 5 minutes
    
    if (error || !signedUrl) {
      return c.json({ error: "Error generating download URL" }, 500);
    }
    
    return c.json({ downloadUrl: signedUrl.signedUrl });
  } catch (error) {
    console.log(`Error generating download URL: ${error}`);
    return c.json({ error: "Error generating download URL" }, 500);
  }
});

Deno.serve(app.fetch);