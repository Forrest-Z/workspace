//Function Declarations
void mergeSort(uchar *numbers,int indices[], int temp[],int temp2[], int array_size);
void m_sort(uchar *numbers,int indices[], int temp[],int temp2[], int left, int right);
void merge(uchar *numbers,int indices[], int temp[],int temp2[], int left, int mid, int right);


void mergeSort(uchar *numbers,int indices[], int temp[],int temp2[], int array_size)
{
  m_sort(numbers,indices, temp,temp2, 0, array_size - 1);
}
 
 
void m_sort(uchar *numbers,int indices[], int temp[],int temp2[], int left, int right)
{
  int mid;
 
  if (right > left)
  {
    mid = (right + left) / 2;
    m_sort(numbers,indices, temp,temp2, left, mid);
    m_sort(numbers,indices, temp,temp2, mid+1, right);
 
    merge(numbers,indices, temp,temp2, left, mid+1, right);
  }
}
 
void merge(uchar *numbers,int indices[], int temp[],int temp2[], int left, int mid, int right)
{
  int i, left_end, num_elements, tmp_pos;
 int tempo;//haha
  left_end = mid - 1;
  tmp_pos = left;
  num_elements = right - left + 1;
 
  while ((left <= left_end) && (mid <= right))
  {
//if (left>25343 || left<0 || mid>25343 || mid<0) {cout<<"first";cin>>tempo;} //haha
    if (int(numbers[left]) <= int(numbers[mid]))
    {
      temp[tmp_pos] = int(numbers[left]);
      temp2[tmp_pos] = indices[left];
      tmp_pos = tmp_pos + 1;
      left = left +1;
    }
    else
    {
      temp[tmp_pos] = int(numbers[mid]);
      temp2[tmp_pos] = indices[mid];

      tmp_pos = tmp_pos + 1;
      mid = mid + 1;
    }
  }
 
  while (left <= left_end)
  {
//if (left>25343 || left<0) {cout<<"second"<<left;cin>>tempo;} //haha
    temp[tmp_pos] = int(numbers[left]);
    temp2[tmp_pos] = indices[left];
    left = left + 1;
    tmp_pos = tmp_pos + 1;
  }
  while (mid <= right)
  {
//if (mid>25343 || mid<0) {cout<<"third";cin>>tempo;} //haha
    temp[tmp_pos] = int(numbers[mid]);
    temp2[tmp_pos] = indices[mid];
    mid = mid + 1;
    tmp_pos = tmp_pos + 1;
  }
 
  for (i=0; i < num_elements; i++)
  {
//if (right>25343 || right<0) {cout<<"fourth"<<right;cin>>tempo;}
    numbers[right] = (uchar)temp[right];
    indices[right] = temp2[right];
    right = right - 1;
  }
}
